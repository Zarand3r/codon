/**
 * @author Richard Bao
 *
 * SlateMemory — the real memory map for a Slate. Materializes the frozen
 * SlateLayout into the live per-shard AlignedBuffers, and provides the byte-level
 * operations the redundancy machinery rides on (hash, deltas, swap, roll).
 * Contracts inferred from SlateMemory.h and the Slate.cc/SlateCombiner call sites.
 */

#include "src/bullwinkle/all/SlateMemory.h"

#include "src/hash/xxh.h"

#include <cstring>
#include <utility>

namespace Drone
{
    /* The shared "no validation" slot every builder compares against. */
    const slate_validator_fn_t slate_no_validation{};

    SlateMemory::SlateMemory(Handle<const SlateLayout> _layout)
        : layout(std::move(_layout))
    {}

    SlateMemory::~SlateMemory() {}

    bool SlateMemory::is_built() const { return is_built_flag; }

    /**
     * Materialize: clone each shard's initial-value template into the live
     * shard table, keep the cyclic templates for roll_frame, and stamp the
     * per-shard layout hash (path/type/offset/size of every element, in index
     * order — identical layout <=> identical hash across strings).
     */
    bool SlateMemory::build()
    {
        FswAbortIf(is_built_flag, false);
        FswAbortIfNot(layout, false);

        for (int s = 0; s < num_slate_shard_t; ++s)
        {
            const AlignedBuffer &init =
                layout->get_initial_values(static_cast<slate_shard_t>(s));
            if (init.size() > 0)
            {
                shard_table[s] = init.clone();
                FswAbortIfNot(shard_table[s].size() == init.size(), false);
            }
        }

        /* One pass over the directory: fold each element into its shard's layout
         * hash (index order) and freeze the per-shard dense region lists the
         * per-cycle delta/hash walks use (offset order = index order per shard,
         * since the allocator hands out ascending offsets). */
        const SlatePathMap &elements = layout->get_elements();
        for (slate_index_t i = 1; i <= elements.size(); ++i)
        {
            const auto it = elements.id_to_iterator(i);
            FswAbortIf(it == elements.end(), false);
            const SlateElementMetadata &m = it->second;
            FswAbortIfNot(m.shard < num_slate_shard_t, false);
            Hash128 &h = shard_layout_hash[m.shard];
            h = digest_xxh128(it->first.data(), it->first.size(), h);
            const UINT64 fields[3] = {m.type_id, m.value_offset, m.value_size};
            h = digest_xxh128(fields, sizeof(fields), h);
            if (!m.is_view_element)
            {
                shard_regions[m.shard].push_back(
                    elem_region_t{m.value_offset, m.value_size});
            }
        }

        cyclic_mem_template = shard_table[shard_cyclic].clone();
        cyclic_mem_no_telem_template = shard_table[shard_cyclic_no_telem].clone();

        is_built_flag = true;
        return true;
    }

    /** Revert the cyclic shards to their templates (start of every frame). */
    bool SlateMemory::roll_frame()
    {
        FswAbortIfNot(is_built_flag, false);
        if (cyclic_mem_template.size() > 0)
        {
            std::memcpy(shard_table[shard_cyclic].data(),
                        cyclic_mem_template.data(), cyclic_mem_template.size());
        }
        if (cyclic_mem_no_telem_template.size() > 0)
        {
            std::memcpy(shard_table[shard_cyclic_no_telem].data(),
                        cyclic_mem_no_telem_template.data(),
                        cyclic_mem_no_telem_template.size());
        }
        return true;
    }

    bool SlateMemory::get_shard_memory(const slate_shard_t shard, B2c &mem) const
    {
        FswAbortIfNot(is_built_flag, false);
        FswAbortIfNot(shard < num_slate_shard_t, false);
        mem = B2c(shard_table[shard].data(), shard_table[shard].size());
        return true;
    }

    bool SlateMemory::get_shard_memory(const slate_shard_t shard, B2 &mem)
    {
        FswAbortIfNot(is_built_flag, false);
        FswAbortIfNot(shard < num_slate_shard_t, false);
        mem = B2(shard_table[shard].data(), shard_table[shard].size());
        return true;
    }

    /** Overwrite a whole shard from a peer image (sizes must match exactly). */
    bool SlateMemory::set_shard_memory(const slate_shard_t shard, const B2c &mem)
    {
        FswAbortIfNot(is_built_flag, false);
        FswAbortIfNot(shard < num_slate_shard_t, false);
        FswAbortIfNot(mem.len() == shard_table[shard].size(), false);
        if (mem.len() > 0)
        {
            std::memcpy(shard_table[shard].data(), mem.buf(), mem.len());
        }
        return true;
    }

    const AlignedBuffer &
    SlateMemory::get_shard_buffer(const slate_shard_t shard) const
    {
        FswDebugAssert(shard < num_slate_shard_t);
        return shard_table[shard];
    }

    /** Install a peer's buffer wholesale (hotsync). Same size; lock must match. */
    bool SlateMemory::swap_shard_buffer(const slate_shard_t shard,
                                        AlignedBuffer &other,
                                        const shard_lock_t lock)
    {
        FswAbortIfNot(is_built_flag, false);
        FswAbortIfNot(shard < num_slate_shard_t, false);
        FswAbortIfNot(lock == shard_lock_table[shard], false);
        FswAbortIfNot(other.size() == shard_table[shard].size(), false);
        std::swap(shard_table[shard], other);
        return true;
    }

    /**
     * Diff two images of this shard's layout: one delta per element whose bytes
     * differ. Regions are element-granular, so a delta pinpoints one element.
     */
    bool SlateMemory::compute_shard_deltas(const slate_shard_t shard,
                                           const B2c mem1, const B2c mem2,
                                           shard_delta_v &deltas,
                                           const size_t max_deltas) const
    {
        FswAbortIfNot(shard < num_slate_shard_t, false);
        FswAbortIfNot(mem1.len() == mem2.len(), false);

        /* Steady state every agreeing frame is "no deltas": one whole-shard
         * memcmp answers that ~90x faster than any per-element walk. */
        if (mem1.len() == 0 ||
            std::memcmp(mem1.buf(), mem2.buf(), mem1.len()) == 0)
        {
            return true;
        }

        for (const elem_region_t &r : shard_regions[shard])
        {
            FswAbortIfNot(r.offset + r.size <= mem1.len(), false);
            const char *p1 = static_cast<const char *>(mem1.buf()) + r.offset;
            const char *p2 = static_cast<const char *>(mem2.buf()) + r.offset;
            if (std::memcmp(p1, p2, r.size) != 0)
            {
                if (deltas.size() >= max_deltas)
                {
                    return true; /* capped; caller asked for at most this many */
                }
                shard_delta_t d;
                d.offset = r.offset;
                d.raw_data[0] = B2c(p1, r.size);
                d.raw_data[1] = B2c(p2, r.size);
                deltas.push_back(d);
            }
        }
        return true;
    }

    bool SlateMemory::is_shard_empty(const slate_shard_t shard) const
    {
        FswAbortIfNot(shard < num_slate_shard_t, true);
        return shard_table[shard].size() == 0;
    }

    bool SlateMemory::get_shard_layout_hash(const slate_shard_t shard,
                                            Hash128 &layout_hash) const
    {
        FswAbortIfNot(is_built_flag, false);
        FswAbortIfNot(shard < num_slate_shard_t, false);
        layout_hash = shard_layout_hash[shard];
        return true;
    }

    bool SlateMemory::wipe_memory()
    {
        for (int s = 0; s < num_slate_shard_t; ++s)
        {
            if (shard_table[s].size() > 0)
            {
                std::memset(shard_table[s].data(), 0, shard_table[s].size());
            }
        }
        return true;
    }

} /* end namespace Drone */

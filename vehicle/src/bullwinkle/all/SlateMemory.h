/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef SLATE_MEMORY_H
#define SLATE_MEMORY_H

#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/SlateLayout.h"
#include "src/bullwinkle/all/core/AlignedBuffer.h"
#include "src/bullwinkle/all/slate_accessor.h"
#include "src/hash/Hash128.h"

namespace Drone
{
    class Slate;
    typedef Slot<bool> slate_validator_fn_t;
    extern const slate_validator_fn_t slate_no_validation;

    /**
     * Describes a single difference between two copies of the same shard layout
     * (e.g. between the sync shard of two peers). A single difference is
     * defined as one or more bit differences within the memory region for a
     * single element.
     */
    struct shard_delta_t
    {
        /**
         * Offset of the data relative to the shard.
         */
        size_t offset = 0;

        /**
         * The raw data for the element in the first and second copy,
         * respectively.
         */
        B2c raw_data[2]{};
    };

    using shard_delta_v = std::vector<shard_delta_t>;

    /**
     * Holds the real memory map for Slate.
     */
    class SlateMemory
    {
    public:
        typedef Signal<bool> signal_t;

        SlateMemory(Handle<const SlateLayout> _layout);
        ~SlateMemory();

        bool build();
        bool is_built() const;

        const void *load_element_r(const slate_element_t element_id,
                                   const slate_type_t type_id) const;

        void *load_element_rw(const slate_element_t element_id,
                              const slate_type_t type_id);

        std::pair<void *, const slate_validator_t>
        load_element_rwv(const slate_element_t element_id,
                         const slate_type_t type_id);

        bool roll_frame();
        bool get_shard_memory(const slate_shard_t shard, B2c &mem) const;
        bool get_shard_memory(const slate_shard_t shard, B2 &mem);
        bool set_shard_memory(const slate_shard_t shard, const B2c &mem);

        const AlignedBuffer &get_shard_buffer(const slate_shard_t shard) const;
        bool swap_shard_buffer(const slate_shard_t shard, AlignedBuffer &other,
                               const shard_lock_t lock = 0);

        bool compute_shard_deltas(const slate_shard_t shard, const B2c mem1,
                                  const B2c mem2, shard_delta_v &deltas,
                                  const size_t max_deltas = -1) const;
        bool is_shard_empty(const slate_shard_t shard) const;
        bool get_shard_layout_hash(const slate_shard_t shard,
                                   Hash128 &layout_hash) const;

        /**
         * @return A read-only reference to SlateLayout paired with this object.
         *
         * N.B. SlateLayout modifications should go through
         * SlateBuilderStoreInterface.
         */
        const SlateLayout &get_layout() const { return *layout; }

        /**
         * Signal which is emitted after build() is called and if the
         * static data is ever modified.
         */
        signal_t validator_sig{};

    private:
        bool wipe_memory();

        /**
         * The layout paiired with this object. This pointer keeps SlateLayout
         * alive if all SlateBuilder instances go out of scope.
         */
        Handle<const SlateLayout> layout{};

        /**
         * True if initialized.
         */
        bool is_built_flag{};

        /**
         * Per-shard hash computed from the layout of the shard.
         */
        Hash128 shard_layout_hash[num_slate_shard_t]{};

        /**
         * Per-shard raw memory packing region.
         */
        AlignedBuffer shard_table[num_slate_shard_t]{};

        /**
         * Per-shard "locks" used to allow/disallow memory swapping.
         */
        shard_lock_t shard_lock_table[num_slate_shard_t]{};

        /**
         * Cyclic shard template. This replaces the existing cyclic shard
         * memory at each frame.
         */
        AlignedBuffer cyclic_mem_template{};

        /**
         * Cyclic shard template. This replaces the existing cyclic shard
         * memory at each frame.
         */
        AlignedBuffer cyclic_mem_no_telem_template{};

    private:
        FSW_DISALLOW_COPY_AND_ASSIGN(SlateMemory);
    };

    /**
     * Get a const pointer to a Slate element.
     *
     * @param element_id ID of the element.
     * @param type_id Expected type of the element.
     *
     * @return The const pointer to the requested Slate element.
     */
    inline const void *
    SlateMemory::load_element_r(const slate_element_t element_id,
                                const slate_type_t type_id) const
    {
        /**
         * Doing all of these branch tests in the final flight build is
         * far too expensive. We include them in dev builds for debugging.
         */
        FswDebugAssert(is_built_flag);
        static_cast<void>(type_id);
        bool may_write = false;
        bool has_validator = false;
        slate_shard_t shard = num_slate_shard_t;
        slate_offset_t offset = 0;
        slate_id_breakdown(element_id, may_write, has_validator, shard, offset);
        FswDebugAssert(shard < num_slate_shard_t);

        const char *value_ptr = shard_table[shard].data() + offset;
        return value_ptr;
    }

    /**
     * Get a non-const pointer to a Slate element.
     *
     * @param element_id ID of the element.
     * @param type_id Expected type of the element.
     *
     * @return The non-const pointer to the requested Slate element.
     */
    inline void *SlateMemory::load_element_rw(const slate_element_t element_id,
                                              const slate_type_t type_id)
    {
        /**
         * Doing all of these branch tests in the final flight build is
         * far too expensive. We include them in dev builds for debugging.
         */
        FswDebugAssert(is_built_flag);
        static_cast<void>(type_id);
        bool may_write = false;
        bool has_validator = false;
        slate_shard_t shard = num_slate_shard_t;
        slate_offset_t offset = 0;
        slate_id_breakdown(element_id, may_write, has_validator, shard, offset);
        FswDebugAssert(may_write);
        FswDebugAssert(!has_validator);
        FswDebugAssert(shard < num_slate_shard_t);

        char *value_ptr = shard_table[shard].data() + offset;
        return value_ptr;
    }

    /**
     * Get a non-const pointer to a Slate element.
     *
     * @param element_id ID of the element.
     * @param type_id Expected type of the element.
     *
     * @return The non-const pointer to the requested Slate element.
     */
    inline std::pair<void *, const slate_validator_t>
    SlateMemory::load_element_rwv(const slate_element_t element_id,
                                  const slate_type_t type_id)
    {
        /**
         * Doing all of these branch tests in the final flight build is
         * far too expensive. We include them in dev builds for debugging.
         */
        FswDebugAssert(is_built_flag);
        static_cast<void>(type_id);
        bool may_write = false;
        bool has_validator = false;
        slate_shard_t shard = num_slate_shard_t;
        slate_offset_t offset = 0;
        slate_id_breakdown(element_id, may_write, has_validator, shard, offset);
        FswDebugAssert(may_write);
        FswDebugAssert(has_validator);
        FswDebugAssert(shard < num_slate_shard_t);

        slate_validator_t validator =
            layout->get_element_validator(element_id, type_id);

        char *value_ptr = shard_table[shard].data() + offset;
        FswDebugAssert(value_ptr);

        return std::pair<void *, const slate_validator_t>(value_ptr, validator);
    };
} /* end namespace Drone*/
#endif /* SLATE_MEMORY_H */

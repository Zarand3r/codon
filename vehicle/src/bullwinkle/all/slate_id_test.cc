// Contract test for slate_id — the packed slate_element_t.
//
// Layout (bit 63..0): [ offset:32 ][ index:26 ][ shard:4 ][ w:1 ][ v:1 ]
//   - slate_id_buildup(out, may_write, has_validator, shard, offset, idx): pack;
//     returns false (no write) if any field exceeds its width.
//   - slate_id_breakdown(id, may_write&, has_validator&, shard&, offset&): unpack
//     (index is fetched separately via slate_id_index — matches call sites).
//   - slate_id_index / _can_write / _has_validator: single-field extractors.
//   - slate_id_ro(id): clear the write bit (read-only view of the element).
//   - slate_id_is_valid(id): a *bound* id (shard < num, and never the default).
//   - slate_id_build_invalid(count): a unique un-bound id from a counter (accountant
//     placeholder); is_valid == false; never equals slate_element_default.
//   - slate_element_default == 0; a bound id is never 0 (SlatePathMap indices >= 1).
// (vehicle/src/bullwinkle/all/slate_id.h)
#include "src/bullwinkle/all/slate_id.h"

#include <cassert>
#include <cstdio>

using namespace Drone;

static void check_roundtrip(bool w, bool v, slate_shard_t shard,
                            slate_offset_t offset, slate_index_t idx)
{
    slate_element_t id = slate_element_default;
    assert(slate_id_buildup(id, w, v, shard, offset, idx));

    bool ow = !w, ov = !v;
    slate_shard_t oshard = shard_invalid;
    slate_offset_t ooff = 0xdead;
    slate_id_breakdown(id, ow, ov, oshard, ooff);
    assert(ow == w);
    assert(ov == v);
    assert(oshard == shard);
    assert(ooff == offset);
    assert(slate_id_index(id) == idx);
    assert(slate_id_can_write(id) == w);
    assert(slate_id_has_validator(id) == v);
    assert(slate_id_is_valid(id)); // idx >= 1 => bound, non-default
}

int main()
{
    // Default sentinel is 0 and is not a valid (bound) id.
    {
        assert(slate_element_default == 0);
        assert(!slate_id_is_valid(slate_element_default));
    }

    // Round-trip every flag combo across boundary field values (idx >= 1).
    {
        const slate_shard_t shards[] = {shard_static, shard_sync, shard_cyclic,
                                        shard_cyclic_no_telem};
        for (const slate_shard_t s : shards)
        {
            check_roundtrip(false, false, s, 0u, 1u);
            check_roundtrip(true, false, s, 1u, 2u);
            check_roundtrip(false, true, s, 4096u, 1000u);
            check_roundtrip(true, true, s, 0xFFFFFFFFu /* max 32b offset */,
                            (1u << 26) - 1 /* max 26b index */);
        }
    }

    // Field overflow / reserved values are rejected (returns false, leaves the
    // out-param untouched).
    {
        slate_element_t id = slate_element_default;
        assert(!slate_id_buildup(id, false, false, shard_static, 0u,
                                 1u << 26)); // index too wide (26 bits)
        assert(id == slate_element_default);
        // A shard out of range is rejected too.
        assert(!slate_id_buildup(id, false, false, shard_invalid, 0u, 1u));
        assert(id == slate_element_default);
        // Index 0 is reserved: rejecting it here is what structurally
        // guarantees a bound id is never the default (0), independent of any
        // caller. This is the invariant slate_id_is_valid relies on.
        assert(!slate_id_buildup(id, false, false, shard_static, 0u, 0u));
        assert(id == slate_element_default);
        assert(!slate_id_buildup(id, true, true, shard_sync, 4096u, 0u));
        assert(id == slate_element_default);
    }

    // slate_id_ro clears the write bit and preserves everything else.
    {
        slate_element_t rw = slate_element_default;
        assert(slate_id_buildup(rw, true /*w*/, false, shard_sync, 512u, 7u));
        assert(slate_id_can_write(rw));

        const slate_element_t ro = slate_id_ro(rw);
        assert(!slate_id_can_write(ro));
        assert(slate_id_is_valid(ro));
        assert(slate_id_index(ro) == 7u);
        bool w = true, v = true;
        slate_shard_t sh = shard_invalid;
        slate_offset_t off = 0;
        slate_id_breakdown(ro, w, v, sh, off);
        assert(!w && !v && sh == shard_sync && off == 512u);
        // Idempotent: ro of a ro is unchanged.
        assert(slate_id_ro(ro) == ro);
    }

    // Invalid (accountant placeholder) ids: unique per count, never valid, never
    // the default, and disjoint from any bound id.
    {
        const slate_element_t a = slate_id_build_invalid(0u);
        const slate_element_t b = slate_id_build_invalid(1u);
        const slate_element_t c = slate_id_build_invalid(123456u);
        assert(a != b && b != c && a != c);
        assert(a != slate_element_default);
        assert(!slate_id_is_valid(a));
        assert(!slate_id_is_valid(b));
        assert(!slate_id_is_valid(c));

        // A bound id in every shard is valid and never collides with invalids.
        for (int s = 0; s < num_slate_shard_t; ++s)
        {
            slate_element_t bound = slate_element_default;
            assert(slate_id_buildup(bound, false, false,
                                    static_cast<slate_shard_t>(s), 0u, 1u));
            assert(slate_id_is_valid(bound));
            assert(bound != a && bound != b && bound != c);
        }
    }

    std::printf("slate_id_test: OK\n");
    return 0;
}

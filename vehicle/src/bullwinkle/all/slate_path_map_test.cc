// Contract test for SlateElementMetadata + SlatePathMap. Inferred from the SlateLayout
// call sites: insert returns {iterator, inserted} (std::map-style); each fresh path
// gets a dense 1-based index; iterator_to_id / id_to_iterator are inverses; duplicate
// paths are rejected without consuming an index; index 0 is reserved (the slate_id
// invariant that keeps a bound id != default).
// (vehicle/src/bullwinkle/all/SlateElement.h, SlatePathMap.h)
#include "src/bullwinkle/all/SlatePathMap.h"

#include <cassert>
#include <cstdio>
#include <string>

using namespace Drone;

static SlateElementMetadata meta(slate_shard_t shard, slate_offset_t off)
{
    return SlateElementMetadata(slate_type_id<double>(), shard, off,
                                sizeof(double), slate_read_write,
                                /*subsystem_id*/ 0, /*is_view*/ false);
}

int main()
{
    // Backing strings for the string_view keys (pool-interned in the real layout).
    const std::string p0 = "nav.altitude_m";
    const std::string p1 = "nav.velocity_mps";
    const std::string p2 = "ctrl.autoseq_begin";

    SlatePathMap m;

    // Empty state.
    assert(m.empty() && m.size() == 0);
    assert(m.find(p0) == m.end());
    assert(m.id_to_iterator(1) == m.end()); // nothing yet

    // First insert -> index 1 (index 0 reserved). This is the slate_id invariant.
    auto r0 = m.insert(std::make_pair(std::string_view(p0), meta(shard_sync, 0)));
    assert(r0.second);                       // inserted
    assert(m.iterator_to_id(r0.first) == 1); // dense, 1-based, never 0
    assert(m.size() == 1);

    auto r1 = m.insert(std::make_pair(std::string_view(p1), meta(shard_nonsync, 8)));
    auto r2 = m.insert(std::make_pair(std::string_view(p2), meta(shard_cyclic, 16)));
    assert(r1.second && r2.second);
    assert(m.iterator_to_id(r1.first) == 2);
    assert(m.iterator_to_id(r2.first) == 3);

    // Duplicate path -> rejected, size and index count unchanged, no index consumed.
    auto dup = m.insert(std::make_pair(std::string_view(p0), meta(shard_static, 99)));
    assert(!dup.second);
    assert(m.iterator_to_id(dup.first) == 1); // still the original element's index
    assert(m.size() == 3);

    // find + iterator_to_id agree.
    assert(m.iterator_to_id(m.find(p1)) == 2);

    // id_to_iterator is the inverse of iterator_to_id, and carries metadata through.
    const auto it2 = m.id_to_iterator(2);
    assert(it2 != m.end());
    assert(it2->first == p1);
    assert(it2->second.shard == shard_nonsync);
    assert(it2->second.value_offset == 8);
    assert(it2->second.type_id == slate_type_id<double>());

    // Round-trip every index.
    for (slate_index_t i = 1; i <= 3; ++i)
    {
        const auto it = m.id_to_iterator(i);
        assert(it != m.end());
        assert(m.iterator_to_id(it) == i);
    }

    // Out-of-range / reserved indices -> end().
    assert(m.id_to_iterator(0) == m.end());   // 0 is reserved, never valid
    assert(m.id_to_iterator(4) == m.end());   // past the end
    assert(m.id_to_iterator(999) == m.end());

    // A built id from a map index is valid (ties the map to slate_id).
    slate_element_t id = slate_element_default;
    assert(slate_id_buildup(id, true, false, shard_sync, 0,
                            m.iterator_to_id(m.find(p0))));
    assert(slate_id_is_valid(id));
    assert(slate_id_index(id) == 1);

    // clear() resets everything, including the index counter.
    m.clear();
    assert(m.empty() && m.find(p0) == m.end() && m.id_to_iterator(1) == m.end());
    auto again = m.insert(std::make_pair(std::string_view(p0), meta(shard_sync, 0)));
    assert(again.second && m.iterator_to_id(again.first) == 1); // counting restarts at 1

    std::printf("slate_path_map_test: OK\n");
    return 0;
}

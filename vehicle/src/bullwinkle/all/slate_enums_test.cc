// Contract test for slate_enums — the Slate shard set, element-access policy, and
// permission algebra. All three are declared-but-undefined imports; the contract is
// inferred from the call sites:
//
//   slate_shard_t        : 7 shards + sentinel; used as an array index and packed
//                          into the element ID (SlateMemory::shard_table[...],
//                          shard_policy[shard]).
//   slate_elem_access_t  : ordered {private < read_only < read_write}; SlateLayout
//                          does std::max(access_policy, elevation) and rejects
//                          access_policy > shard_policy[shard].
//   slate_permission_t   : bitmask; slate_permission_deny(base, mask),
//                          slate_permission_is_superset (no privilege elevation in a
//                          sub-slate), slate_permission_take_subset (effective =
//                          parent & requested), slate_can_read/write/create.
//   *_sym                : SymbolTable reflection (get(value) -> name).
// (vehicle/src/bullwinkle/all/slate_enums.h)
#include "src/bullwinkle/all/slate_enums.h"

#include <cassert>
#include <cstdio>

using namespace Drone;

int main()
{
    // --- Shard set ---------------------------------------------------------
    {
        // Exactly seven shards, all distinct, all valid indices < num.
        assert(num_slate_shard_t == 7);
        const slate_shard_t all[] = {
            shard_static,  shard_sync,   shard_nonsync,        shard_cyclic,
            shard_sync_no_telem, shard_nonsync_no_telem, shard_cyclic_no_telem};
        for (const slate_shard_t s : all)
        {
            assert(s < num_slate_shard_t);
        }
        // The sentinel is out of the valid range (so `shard < num` rejects it).
        assert(shard_invalid >= num_slate_shard_t);

        // Reflection round-trips.
        assert(slate_shard_t_sym.get(shard_sync) == "sync");
        assert(slate_shard_t_sym.get(shard_nonsync) == "nonsync");
        assert(slate_shard_t_sym.get(shard_static) == "static");
        uint v = 0;
        assert(slate_shard_t_sym.raw_get("cyclic", v) && v == shard_cyclic);
    }

    // --- Element access policy (ordered) -----------------------------------
    {
        assert(slate_private < slate_read_only);
        assert(slate_read_only < slate_read_write);
        assert(slate_elem_access_t_sym.get(slate_read_only) == "read_only");
        assert(slate_elem_access_t_sym.get(slate_read_write) == "read_write");
    }

    // --- Permission algebra ------------------------------------------------
    {
        // r: read only.
        assert(slate_can_read(slate_permission_r));
        assert(!slate_can_write(slate_permission_r));
        assert(!slate_can_create(slate_permission_r, shard_nonsync));
        assert(!slate_can_create(slate_permission_r, shard_sync));

        // rwc: everything, in every shard class.
        assert(slate_can_read(slate_permission_rwc));
        assert(slate_can_write(slate_permission_rwc));
        assert(slate_can_create(slate_permission_rwc, shard_sync));
        assert(slate_can_create(slate_permission_rwc, shard_sync_no_telem));
        assert(slate_can_create(slate_permission_rwc, shard_nonsync));

        // Runtime = rwc minus sync-create: create in nonsync but never in sync.
        const slate_permission_t runtime =
            slate_permission_deny(slate_permission_rwc, slate_permission_c_sync);
        assert(slate_can_write(runtime));
        assert(slate_can_create(runtime, shard_nonsync));
        assert(slate_can_create(runtime, shard_cyclic));
        assert(!slate_can_create(runtime, shard_sync));
        assert(!slate_can_create(runtime, shard_sync_no_telem));

        // Control = rwc minus nonsync-create: create in sync but never in nonsync.
        const slate_permission_t control = slate_permission_deny(
            slate_permission_rwc, slate_permission_c_nonsync);
        assert(slate_can_create(control, shard_sync));
        assert(!slate_can_create(control, shard_nonsync));
        assert(!slate_can_create(control, shard_cyclic));

        // rc: read + create (no write), in both classes.
        assert(slate_can_read(slate_permission_rc));
        assert(!slate_can_write(slate_permission_rc));
        assert(slate_can_create(slate_permission_rc, shard_sync));
        assert(slate_can_create(slate_permission_rc, shard_nonsync));

        // Superset: a sub-slate may not elevate privileges.
        assert(slate_permission_is_superset(slate_permission_rwc,
                                            slate_permission_r));
        assert(slate_permission_is_superset(slate_permission_rwc, runtime));
        assert(slate_permission_is_superset(slate_permission_rwc,
                                            slate_permission_rwc));
        assert(!slate_permission_is_superset(slate_permission_r,
                                             slate_permission_rwc));
        assert(!slate_permission_is_superset(runtime, control)); // neither ⊇ other

        // take_subset: effective = parent & requested.
        assert(slate_permission_take_subset(slate_permission_rwc,
                                            slate_permission_r) ==
               slate_permission_r);
        const slate_permission_t eff =
            slate_permission_take_subset(runtime, slate_permission_rc);
        // runtime ∩ rc = read + create + create_nonsync (no write, no sync-create).
        assert(slate_can_read(eff));
        assert(!slate_can_write(eff));
        assert(slate_can_create(eff, shard_nonsync));
        assert(!slate_can_create(eff, shard_sync));
    }

    std::printf("slate_enums_test: OK\n");
    return 0;
}

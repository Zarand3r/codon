// GOLDEN-PATH L1 integration test (IMPLEMENTATION_PLAN §4.1/§4.3) — the P1
// acceptance gate: build a Slate through the real store, create elements across
// shards via tokens, freeze, store/load through the handle layer, hash, diff.
#include "src/bullwinkle/all/Slate.h"
#include "src/bullwinkle/all/SlateBuilder.h"
#include "src/bullwinkle/all/SlateBuilderStore.h"

#include <cassert>
#include <vector>
#include <cstdio>

using namespace Drone;

int main()
{
    SlateBuilder builder(CreateSlateBuilderStore());

    // Create one element in every shard through the typed handle layer.
    WriteToken<double> alt;
    WriteToken<INT64> ticks;
    WriteToken<bool> flag;
    ReadToken<INT32> constant;
    assert(builder.create("nav.altitude_m", 0.0, shard_sync, slate_read_write, alt));
    assert(builder.create("nav.ticks", (INT64)0, shard_nonsync, slate_read_write, ticks));
    assert(builder.create("ctrl.reset", false, shard_cyclic, slate_read_write, flag));
    assert(builder.create("cfg.version", (INT32)7, shard_static, slate_read_only, constant));
    WriteToken<double> a2, a3, a4;
    assert(builder.create("t.sync_nt", 1.0, shard_sync_no_telem, slate_read_write, a2));
    assert(builder.create("t.nonsync_nt", 2.0, shard_nonsync_no_telem, slate_read_write, a3));
    assert(builder.create("t.cyclic_nt", 3.0, shard_cyclic_no_telem, slate_read_write, a4));

    // Freeze + materialize.
    Slate slate = builder.slate(slate_no_validation);
    assert(builder.build());

    // P2: create() after freeze is rejected.
    WriteToken<double> late;
    assert(!builder.create("late.elem", 0.0, shard_sync, slate_read_write, late));

    // Store/load round-trip through tokens (initial values then updates).
    assert(slate[constant] == 7);
    assert(slate[a2] == 1.0);
    slate[alt] = 42.5;
    assert(slate[alt] == 42.5);
    slate[ticks] = (INT64)-9;
    assert(slate[ticks] == (INT64)-9);
    slate[flag] = true;
    assert(slate[flag]);

    // P4: hash equal for identical state; delta pinpoints one mutation.
    UINT64 h1 = 0, h2 = 0;
    assert(slate.compute_hash(shard_sync, h1));
    assert(slate.compute_hash(shard_sync, h2));
    assert(h1 == h2);
    slate[alt] = 43.0; // one mutation
    assert(slate.compute_hash(shard_sync, h2));
    assert(h1 != h2);

    // Deltas pinpoint exactly the mutated element: capture the sync shard image,
    // mutate one element, diff old vs new.
    {
        B2c live;
        assert(slate.get_shard_memory(shard_sync, live));
        std::vector<char> snapshot(static_cast<const char *>(live.buf()),
                                   static_cast<const char *>(live.buf()) +
                                       live.len());
        slate[alt] = 44.25; // exactly one element changes
        shard_delta_v deltas;
        assert(slate.compute_shard_deltas(
            shard_sync, B2c(snapshot.data(), snapshot.size()), live, deltas));
        assert(deltas.size() == 1);
        assert(deltas[0].raw_data[0].len() == sizeof(double));
        // Identical images -> zero deltas (the steady-state fast path).
        shard_delta_v none;
        assert(slate.compute_shard_deltas(shard_sync, live, live, none));
        assert(none.empty());
    }

    // roll_frame: cyclic reverts to its template; sync persists.
    {
        assert(slate[flag]);           // set true above
        assert(slate.roll_frame());
        assert(!slate[flag]);          // cyclic reverted to initial (false)
        assert(slate[a4] == 3.0);      // cyclic_no_telem reverted too
        assert(slate[alt] == 44.25);   // sync persisted across the frame
    }

    std::printf("golden-path L1: OK\n");
    return 0;
}

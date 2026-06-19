#pragma once

// Hash128: a 128-bit digest value — both the layout-hash identity and the
// content-hash result used to compare Slate shards byte-for-byte across units.
// Contract from usage: union-of-two-`UINT64` field `u64[2]` (folded via
// `u64[0] ^ u64[1]` in `Slate::compute_hash`); default-constructible; comparable.

#include "src/bullwinkle/all/core/drone_types.h"

namespace Drone
{
    struct Hash128
    {
        UINT64 u64[2];

        Hash128() : u64{0, 0} {}
        Hash128(UINT64 lo, UINT64 hi) : u64{lo, hi} {}

        bool operator==(const Hash128 &o) const
        {
            return u64[0] == o.u64[0] && u64[1] == o.u64[1];
        }
        bool operator!=(const Hash128 &o) const { return !(*this == o); }
    };
} // namespace Drone

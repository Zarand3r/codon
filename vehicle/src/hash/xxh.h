#pragma once

// digest_xxh128: deterministic 128-bit content hash, seeded.
// Contract from usage: `Hash128 digest_xxh128(const void* buf, size_t len, Hash128 seed)`
// (Slate::compute_hash seeds the content hash with the shard's layout hash).
//
// Implemented with MurmurHash3_x64_128 (public-domain, Austin Appleby). The only
// requirement here is determinism and *identical output across units running the
// same build* — the hash is used to compare a unit's Slate shard against a peer's,
// never to interoperate with an external party. If ground-tool hash compatibility
// is ever required, swap in canonical XXH3-128 behind this same signature.

#include "src/bullwinkle/all/core/drone_types.h"
#include "src/hash/Hash128.h"

#include <cstddef>
#include <cstring>

namespace Drone
{
    namespace xxh_detail
    {
        inline UINT64 rotl64(UINT64 x, int r)
        {
            return (x << r) | (x >> (64 - r));
        }
        inline UINT64 fmix64(UINT64 k)
        {
            k ^= k >> 33;
            k *= 0xff51afd7ed558ccdULL;
            k ^= k >> 33;
            k *= 0xc4ceb9fe1a85ec53ULL;
            k ^= k >> 33;
            return k;
        }
        inline UINT64 load64(const UINT8 *p)
        {
            UINT64 v;
            std::memcpy(&v, p, sizeof(v)); // alignment-safe; little-endian target
            return v;
        }
    } // namespace xxh_detail

    inline Hash128 digest_xxh128(const void *key, size_t len, Hash128 seed)
    {
        using namespace xxh_detail;
        const UINT8 *data = static_cast<const UINT8 *>(key);
        const size_t nblocks = len / 16;

        UINT64 h1 = seed.u64[0];
        UINT64 h2 = seed.u64[1];
        const UINT64 c1 = 0x87c37b91114253d5ULL;
        const UINT64 c2 = 0x4cf5ad432745937fULL;

        for (size_t i = 0; i < nblocks; ++i)
        {
            UINT64 k1 = load64(data + i * 16 + 0);
            UINT64 k2 = load64(data + i * 16 + 8);

            k1 *= c1; k1 = rotl64(k1, 31); k1 *= c2; h1 ^= k1;
            h1 = rotl64(h1, 27); h1 += h2; h1 = h1 * 5 + 0x52dce729;
            k2 *= c2; k2 = rotl64(k2, 33); k2 *= c1; h2 ^= k2;
            h2 = rotl64(h2, 31); h2 += h1; h2 = h2 * 5 + 0x38495ab5;
        }

        const UINT8 *tail = data + nblocks * 16;
        UINT64 k1 = 0;
        UINT64 k2 = 0;
        switch (len & 15)
        {
        case 15: k2 ^= static_cast<UINT64>(tail[14]) << 48; // fallthrough
        case 14: k2 ^= static_cast<UINT64>(tail[13]) << 40; // fallthrough
        case 13: k2 ^= static_cast<UINT64>(tail[12]) << 32; // fallthrough
        case 12: k2 ^= static_cast<UINT64>(tail[11]) << 24; // fallthrough
        case 11: k2 ^= static_cast<UINT64>(tail[10]) << 16; // fallthrough
        case 10: k2 ^= static_cast<UINT64>(tail[9]) << 8;   // fallthrough
        case 9:
            k2 ^= static_cast<UINT64>(tail[8]) << 0;
            k2 *= c2; k2 = rotl64(k2, 33); k2 *= c1; h2 ^= k2;
            // fallthrough
        case 8: k1 ^= static_cast<UINT64>(tail[7]) << 56; // fallthrough
        case 7: k1 ^= static_cast<UINT64>(tail[6]) << 48; // fallthrough
        case 6: k1 ^= static_cast<UINT64>(tail[5]) << 40; // fallthrough
        case 5: k1 ^= static_cast<UINT64>(tail[4]) << 32; // fallthrough
        case 4: k1 ^= static_cast<UINT64>(tail[3]) << 24; // fallthrough
        case 3: k1 ^= static_cast<UINT64>(tail[2]) << 16; // fallthrough
        case 2: k1 ^= static_cast<UINT64>(tail[1]) << 8;  // fallthrough
        case 1:
            k1 ^= static_cast<UINT64>(tail[0]) << 0;
            k1 *= c1; k1 = rotl64(k1, 31); k1 *= c2; h1 ^= k1;
            break;
        default: break;
        }

        h1 ^= static_cast<UINT64>(len);
        h2 ^= static_cast<UINT64>(len);
        h1 += h2;
        h2 += h1;
        h1 = fmix64(h1);
        h2 = fmix64(h2);
        h1 += h2;
        h2 += h1;
        return Hash128(h1, h2);
    }
} // namespace Drone

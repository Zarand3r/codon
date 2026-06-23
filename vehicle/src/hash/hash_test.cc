// Contract test for Hash128 + digest_xxh128: determinism, sensitivity, avalanche.
#include "src/hash/Hash128.h"
#include "src/hash/xxh.h"

#include <cassert>
#include <cstdio>
#include <cstring>

using Drone::digest_xxh128;
using Drone::Hash128;

static Hash128 h(const char *s, Hash128 seed = Hash128())
{
    return digest_xxh128(s, std::strlen(s), seed);
}

// Number of differing bits between two 128-bit digests.
static int hamming(Hash128 a, Hash128 b)
{
    return __builtin_popcountll(a.u64[0] ^ b.u64[0]) +
           __builtin_popcountll(a.u64[1] ^ b.u64[1]);
}

int main()
{
    // Value type: zero default, comparable.
    assert((Hash128() == Hash128(0, 0)));
    assert((Hash128(1, 2) != Hash128(2, 1)));

    // Determinism: same input + seed -> identical digest.
    assert(h("the quick brown fox") == h("the quick brown fox"));

    // Content sensitivity.
    assert(h("abc") != h("abd")); // one char
    assert(h("") != h("x"));      // empty vs non-empty

    // Seed sensitivity: same content, different 128-bit seed -> different digest.
    assert(h("payload", Hash128(1, 0)) != h("payload", Hash128(2, 0)));

    // Multi-block (>16 bytes) path differs from a 16-byte prefix.
    const char *big = "0123456789ABCDEF0123456789ABCDEF!"; // 33 bytes
    assert(digest_xxh128(big, 33, Hash128()) != digest_xxh128(big, 16, Hash128()));

    // Empty input is handled (no crash) and deterministic.
    assert(digest_xxh128(nullptr, 0, Hash128(7, 9)) ==
           digest_xxh128(nullptr, 0, Hash128(7, 9)));

    // Avalanche: flipping a single input bit must diffuse to ~half of the 128
    // output bits. A broken mix (poor diffusion or a fixed output) would land far
    // outside [32, 96]; a correct hash sits near 64. Deterministic inputs -> no flake.
    unsigned char buf[32];
    for (int i = 0; i < 32; ++i)
        buf[i] = static_cast<unsigned char>(i * 7 + 1);
    const Hash128 base = digest_xxh128(buf, sizeof(buf), Hash128());
    for (int bit = 0; bit < 8; ++bit) // flip several distinct bits
    {
        buf[bit] ^= static_cast<unsigned char>(1u << bit);
        const int d = hamming(base, digest_xxh128(buf, sizeof(buf), Hash128()));
        assert(d >= 32 && d <= 96);
        buf[bit] ^= static_cast<unsigned char>(1u << bit); // restore
    }

    std::printf("hash_test: PASS\n");
    return 0;
}

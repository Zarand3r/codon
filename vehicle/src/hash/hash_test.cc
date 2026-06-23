// TDD (P0): Hash128 + digest_xxh128 contract — determinism, sensitivity, folding.
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

int main()
{
    // Default Hash128 is zero and comparable.
    assert((Hash128() == Hash128(0, 0)));
    assert((Hash128(1, 2) != Hash128(2, 1)));

    // Determinism: same input + seed → identical digest.
    assert(h("the quick brown fox") == h("the quick brown fox"));

    // Content sensitivity: one-bit/one-char change → different digest.
    assert(h("abc") != h("abd"));
    assert(h("") != h("x"));

    // Seed sensitivity: same content, different seed → different digest.
    assert(h("payload", Hash128(1, 0)) != h("payload", Hash128(2, 0)));

    // Multi-block (>16 bytes) path differs from a prefix.
    const char *big = "0123456789ABCDEF0123456789ABCDEF!"; // 33 bytes
    assert(digest_xxh128(big, 33, Hash128()) !=
           digest_xxh128(big, 16, Hash128()));

    // Empty input is handled deterministically.
    assert(digest_xxh128(nullptr, 0, Hash128(7, 9)) ==
           digest_xxh128(nullptr, 0, Hash128(7, 9)));

    // The fold Slate uses (u64[0]^u64[1]) is stable across calls.
    const Hash128 d = h("fold-check");
    assert((d.u64[0] ^ d.u64[1]) == (h("fold-check").u64[0] ^ h("fold-check").u64[1]));

    std::printf("hash_test: PASS\n");
    return 0;
}

// TDD (P1): contract test for core/util.h, written before the implementation.
// Contract inferred from present usage:
//   MonotonicPool: `void* allocate(size_t size, size_t alignment)` + `release()`
//     (SlateLayout.h:150 `MonotonicPool string_pool;`,
//      SlateLayout.cc:797 `string_pool.allocate(_path.size(), 1)`, :767 `release()`)
//   join(const str_v&) -> std::string  (SlateCombiner.cc:185 `join(line).c_str()`)
//   typedefs str_v / str_s / str_v_v
#include "src/bullwinkle/all/core/util.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

using Drone::join;
using Drone::MonotonicPool;
using Drone::str_s;
using Drone::str_v;
using Drone::str_v_v;

static bool aligned(const void *p, std::size_t a)
{
    return (reinterpret_cast<std::uintptr_t>(p) & (a - 1)) == 0;
}

int main()
{
    // join: space-separated.
    assert(join(str_v{}) == "");
    assert(join(str_v{"x"}) == "x");
    assert(join(str_v{"a", "b", "c"}) == "a b c");

    // typedefs are the expected containers.
    str_s s;
    s.insert("p");
    s.insert("p");
    assert(s.size() == 1); // set dedups
    str_v_v vv;
    vv.push_back(str_v{"a", "b"});
    assert(vv[0][1] == "b");

    MonotonicPool pool;

    // alignment honored; allocations are distinct and writable.
    void *a = pool.allocate(10, 1);
    void *b = pool.allocate(8, 8);
    assert(a && b && a != b);
    assert(aligned(b, 8));
    std::memset(a, 0xAA, 10);
    std::memset(b, 0xBB, 8);
    assert(static_cast<unsigned char *>(a)[0] == 0xAA);
    assert(static_cast<unsigned char *>(b)[0] == 0xBB);

    // allocation larger than the default chunk works and is aligned.
    void *big = pool.allocate(1u << 16, 64);
    assert(big && aligned(big, 64));
    std::memset(big, 0xCC, 1u << 16);

    // many small allocations never overlap: stamp a unique int in each, read back.
    const int kN = 1000;
    int *slots[kN];
    for (int i = 0; i < kN; ++i)
    {
        slots[i] = static_cast<int *>(pool.allocate(sizeof(int), alignof(int)));
        assert(aligned(slots[i], alignof(int)));
        *slots[i] = i;
    }
    for (int i = 0; i < kN; ++i)
        assert(*slots[i] == i); // none clobbered -> no overlap

    // release frees everything; the pool is reusable afterward.
    pool.release();
    void *c = pool.allocate(16, 16);
    assert(c && aligned(c, 16));

    // size-0 allocation returns a usable aligned pointer (empty path case).
    void *z = pool.allocate(0, 1);
    assert(z != nullptr);

    std::printf("util_test: PASS\n");
    return 0;
}

// Contract test for slate_type — the deterministic, cross-string-stable type id.
//
// Contract inferred from usage:
//   slate_type_t        : 64-bit id (SlateLayout does
//                         FswAbortIfEqUint64(type_id, slate_type_invalid, ...)).
//   slate_type_invalid  : the 0 sentinel; a real type never hashes to it.
//   slate_type_id<T>()  : hash of the compiler's per-type name (__PRETTY_FUNCTION__)
//                         via digest_xxh128 — deterministic for a fixed binary, so
//                         all three strings (identical binary) agree; distinct types
//                         get distinct ids (SlateToken<T>::get_type_id, bind checks).
// (vehicle/src/bullwinkle/all/slate_type.h)
#include "src/bullwinkle/all/slate_type.h"

#include <cassert>
#include <cstdio>
#include <set>

using namespace Drone;

namespace
{
    struct Alpha
    {
        int x;
    };
    struct Beta
    {
        int x;
    };
    template <typename A, typename B>
    struct Pair
    {
    };
} // namespace

int main()
{
    // Stable within a run: same type -> same id, every call.
    {
        assert(slate_type_id<int>() == slate_type_id<int>());
        assert(slate_type_id<Alpha>() == slate_type_id<Alpha>());
    }

    // Never the invalid sentinel.
    {
        assert(slate_type_id<int>() != slate_type_invalid);
        assert(slate_type_id<Alpha>() != slate_type_invalid);
        assert(slate_type_invalid == static_cast<slate_type_t>(0));
    }

    // Distinct types get distinct ids — including look-alikes (same layout, diff
    // name), cv/ref/pointer variants, and template instantiations.
    {
        std::set<slate_type_t> ids;
        ids.insert(slate_type_id<int>());
        ids.insert(slate_type_id<unsigned>());
        ids.insert(slate_type_id<long>());
        ids.insert(slate_type_id<double>());
        ids.insert(slate_type_id<float>());
        ids.insert(slate_type_id<bool>());
        ids.insert(slate_type_id<char>());
        ids.insert(slate_type_id<Alpha>());
        ids.insert(slate_type_id<Beta>()); // same members as Alpha, different type
        ids.insert(slate_type_id<int *>());
        ids.insert(slate_type_id<const int>());
        ids.insert(slate_type_id<Pair<int, double>>());
        ids.insert(slate_type_id<Pair<double, int>>());
        // 13 distinct types -> 13 distinct ids (no collision at 64 bits).
        assert(ids.size() == 13);
    }

    // Alpha and Beta are structurally identical but must not alias.
    {
        assert(slate_type_id<Alpha>() != slate_type_id<Beta>());
    }

    std::printf("slate_type_test: OK\n");
    return 0;
}

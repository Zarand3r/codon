// Contract test for the slate_info<T> value trait (the bridge between the typed API
// and the byte-oriented shard). This is a *behavioral* test: it drives values through
// the exact members the Slate/SlateBuilder call sites use — construct into raw
// (aligned) memory, read back via from_mem, overwrite via copy — matching the real
// build-then-runtime write paths.
//   build:   allocate_element -> slate_info<T>::construct(mem, initial_value)
//   runtime: slate_info<T>::copy(value, load_rw<T>(id))  (dst is from_mem(void*))
//   read:    slate_info<T>::from_mem(const void*) -> R
// (vehicle/src/bullwinkle/all/slate_info.h)
#include "src/bullwinkle/all/slate_info.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>

using namespace Drone;

namespace
{
    struct Pose // trivially-copyable POD aggregate
    {
        double x;
        double y;
        INT64 stamp;
        bool operator==(const Pose &o) const
        {
            return x == o.x && y == o.y && stamp == o.stamp;
        }
    };
} // namespace

template <typename T>
static void exercise(const T &initial, const T &updated)
{
    // Aligned backing "shard slot" of the trait's reported size/alignment.
    alignas(alignof(T)) unsigned char buf[sizeof(T) + 32];
    void *mem = buf;

    // size()/alignment() report the type's footprint.
    T probe = initial;
    assert(slate_info<T>::size(probe) == sizeof(T));
    assert(slate_info<T>::alignment() == alignof(T));

    // build-phase write: construct the initial value in place.
    assert(slate_info<T>::construct(mem, initial));

    // read view: from_mem(const void*) returns the value just placed.
    typename slate_info<T>::R r = slate_info<T>::from_mem(static_cast<const void *>(mem));
    assert(r == initial);

    // runtime write: copy() overwrites through the writable view.
    typename slate_info<T>::W w = slate_info<T>::from_mem(mem);
    assert(slate_info<T>::copy(updated, w));
    assert(slate_info<T>::from_mem(static_cast<const void *>(mem)) == updated);

    // The write actually landed in `buf` at offset 0 (byte-level check).
    T readback;
    std::memcpy(&readback, buf, sizeof(T));
    assert(readback == updated);
}

int main()
{
    // Typed views are what the call sites expect (R const-ref, W mutable-ref).
    static_assert(std::is_same<slate_info<double>::R, const double &>::value, "R");
    static_assert(std::is_same<slate_info<double>::W, double &>::value, "W");
    static_assert(std::is_same<slate_info<double>::O, double &>::value, "O");

    // is_valid gate: POD scalars/structs OK; pointers and non-trivially-copyable out.
    static_assert(slate_info<double>::is_valid(), "double storable");
    static_assert(slate_info<INT64>::is_valid(), "INT64 storable");
    static_assert(slate_info<bool>::is_valid(), "bool storable");
    static_assert(slate_info<Pose>::is_valid(), "POD struct storable");
    static_assert(!slate_info<double *>::is_valid(), "pointer rejected");
    static_assert(!slate_info<std::string>::is_valid(), "std::string rejected");

    // type_id delegates to the stable type id; distinct types differ.
    assert(slate_info<double>::type_id() == slate_type_id<double>());
    assert(slate_info<double>::type_id() != slate_info<INT64>::type_id());

    // Behavioral round-trips through the real build/runtime member paths.
    exercise<double>(3.14, -2.5);
    exercise<INT64>(42, -7);
    exercise<bool>(true, false);
    exercise<Pose>(Pose{1.0, 2.0, 100}, Pose{9.0, 8.0, 200});

    std::printf("slate_info_test: OK\n");
    return 0;
}

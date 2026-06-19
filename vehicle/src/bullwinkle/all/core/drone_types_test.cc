// TDD (P0): drone_types contract — fixed widths + copy-suppression macro.
#include "src/bullwinkle/all/core/drone_types.h"

#include <cstdio>
#include <type_traits>

static_assert(sizeof(UINT8) == 1, "UINT8 is 1 byte");
static_assert(sizeof(UINT16) == 2, "UINT16 is 2 bytes");
static_assert(sizeof(UINT32) == 4, "UINT32 is 4 bytes");
static_assert(sizeof(UINT64) == 8, "UINT64 is 8 bytes");
static_assert(sizeof(INT8) == 1 && sizeof(INT64) == 8, "INT widths");
static_assert(std::is_unsigned<UINT64>::value, "UINT64 unsigned");
static_assert(std::is_signed<INT64>::value, "INT64 signed");
static_assert(std::is_same<uint, unsigned int>::value, "uint == unsigned int");

namespace
{
    struct NonCopyable
    {
        NonCopyable() = default;
        SX_DISALLOW_COPY_AND_ASSIGN(NonCopyable);
    };
    static_assert(!std::is_copy_constructible<NonCopyable>::value,
                  "SX_DISALLOW_COPY_AND_ASSIGN deletes the copy ctor");
    static_assert(!std::is_copy_assignable<NonCopyable>::value,
                  "SX_DISALLOW_COPY_AND_ASSIGN deletes copy-assign");
}

int main()
{
    std::printf("drone_types_test: PASS\n");
    return 0;
}

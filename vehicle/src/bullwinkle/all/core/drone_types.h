#pragma once

// Fixed-width scalar typedefs + copy-suppression macro. These make data structures
// byte-portable across machines/reboots (the Slate's position-independent
// requirement). Defined at global scope to match present unqualified usage
// (`UINT8`, `INT64`, `uint`) across the tree.
//
// Contract from usage: `UINT8..64`, `INT8..64`, `uint`, `FSW_DISALLOW_COPY_AND_ASSIGN`.

#include <cstdint>

using UINT8 = uint8_t;
using UINT16 = uint16_t;
using UINT32 = uint32_t;
// 64-bit types are the `long long` family (not uint64_t, which is `unsigned long`
// on LP64). The imported consumers print 64-bit values with the `ll` length
// modifier (e.g. `dbnprintf(": id: 0x%016llx", slate_element_t)`, `%lld`), so under
// -Werror=format the underlying type must be `[unsigned] long long`. Still exactly
// 64-bit (asserted below); this also keeps the `ll` specifiers portable to LLP64.
using UINT64 = unsigned long long;

using INT8 = int8_t;
using INT16 = int16_t;
using INT32 = int32_t;
using INT64 = long long;

static_assert(sizeof(UINT64) == 8, "UINT64 must be exactly 64-bit");
static_assert(sizeof(INT64) == 8, "INT64 must be exactly 64-bit");

// `unsigned int` alias. Identical to the common <sys/types.h> `uint`, so the
// duplicate typedef is well-formed in C++ when both are visible.
using uint = unsigned int;

// Delete the copy constructor and copy-assignment of a class (place in the
// class body). Used by move-only/owning types (AlignedBuffer, SlateMemory, …).
#define FSW_DISALLOW_COPY_AND_ASSIGN(ClassName)                                 \
    ClassName(const ClassName &) = delete;                                     \
    ClassName &operator=(const ClassName &) = delete

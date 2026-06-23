// Contract test for AlignedBuffer — the per-shard heap storage SlateMemory holds.
// Contract from the header/impl: default-empty; ensure(amount, alignment) grows the
// buffer (geometric), keeps it max_alignment-aligned, preserves old content and
// zero-fills the new region; length only grows; clone() is a deep copy; move-only
// with source emptied. (vehicle/src/bullwinkle/all/core/AlignedBuffer.h)
#include "src/bullwinkle/all/core/AlignedBuffer.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>

using Drone::AlignedBuffer;

static bool is_aligned(const void *p, std::size_t a)
{
    return (reinterpret_cast<std::uintptr_t>(p) % a) == 0;
}

int main()
{
    // Default: empty, not allocated.
    {
        AlignedBuffer b;
        assert(b.data() == nullptr && b.size() == 0 && b.capacity() == 0);
        assert(!b); // operator bool
    }

    // ensure() allocates, aligns (to max_alignment=32, which satisfies any <=32),
    // sizes, and yields writable storage.
    {
        AlignedBuffer b;
        assert(b.ensure(100, 8));
        assert(static_cast<bool>(b));
        assert(b.data() != nullptr);
        assert(is_aligned(b.data(), AlignedBuffer::max_alignment));
        assert(b.size() == 100 && b.capacity() >= 100);
        std::memset(b.data(), 0xAB, b.size());
        assert(static_cast<unsigned char>(b.data()[99]) == 0xAB);
    }

    // Invalid alignment is rejected (0, non-power-of-two, > max_alignment).
    {
        AlignedBuffer b;
        assert(!b.ensure(10, 0));
        assert(!b.ensure(10, 3));
        assert(!b.ensure(10, 64));
        assert(b.size() == 0); // unchanged
    }

    // Grow preserves existing content and zero-fills the new region.
    {
        AlignedBuffer b;
        assert(b.ensure(64, 8));
        for (std::size_t i = 0; i < 64; ++i)
            b.data()[i] = static_cast<char>(i + 1);
        assert(b.ensure(4096, 8)); // forces reallocation
        assert(b.size() == 4096 && b.capacity() >= 4096);
        for (std::size_t i = 0; i < 64; ++i)
            assert(static_cast<unsigned char>(b.data()[i]) ==
                   static_cast<unsigned char>(i + 1)); // preserved
        for (std::size_t i = 64; i < 4096; ++i)
            assert(b.data()[i] == 0); // new region zeroed
    }

    // ensure(<= capacity) keeps the buffer; length only grows (never shrinks).
    {
        AlignedBuffer b;
        assert(b.ensure(200, 8));
        const char *p = b.data();
        assert(b.ensure(50, 8));
        assert(b.data() == p);   // no reallocation
        assert(b.size() == 200); // not shrunk to 50
    }

    // clone() is an independent deep copy sized to the source length.
    {
        AlignedBuffer a;
        assert(a.ensure(128, 8));
        std::memset(a.data(), 0x5A, 128);
        AlignedBuffer c = a.clone();
        assert(c.size() == 128 && c.data() != a.data());
        assert(std::memcmp(a.data(), c.data(), 128) == 0);
        a.data()[0] = 0x00;
        assert(static_cast<unsigned char>(c.data()[0]) == 0x5A); // unaffected
    }

    // Move-only: source is emptied, destination owns the storage.
    {
        AlignedBuffer a;
        assert(a.ensure(80, 8));
        std::memset(a.data(), 0x33, 80);
        const char *moved = a.data();
        AlignedBuffer b(std::move(a));
        assert(b.data() == moved && b.size() == 80);
        assert(static_cast<unsigned char>(b.data()[0]) == 0x33);
        assert(a.data() == nullptr && a.size() == 0 && !a);

        AlignedBuffer d;
        d = std::move(b);
        assert(d.size() == 80 && static_cast<unsigned char>(d.data()[0]) == 0x33);
        assert(b.data() == nullptr && !b);
    }

    std::printf("aligned_buffer_test: PASS\n");
    return 0;
}

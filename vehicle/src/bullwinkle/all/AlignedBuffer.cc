/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#include "src/bullwinkle/all/core/AlignedBuffer.h"

#include "src/bullwinkle/all/core/sac.h"

#include <cstdint>
#include <cstring>
#include <new>
#include <utility>

namespace Drone
{
    /*
     * Allocate 3/4 of a typical OS page initially to account for any book
     * keeping structures.
     */
    static constexpr size_t initial_total = 3072;

    /**
     * Allocate an aligned buffer.
     *
     * We align the allocated pointer manually instead of using the C++17
     * aligned new[] and delete[] to avoid a bad interaction between Valgrind
     * and the heap tracker. It looks like the aligned allocations are routed
     * via the heap tracker:
     *
     *   ==23==    at 0x483264B: malloc (vg_replace_malloc.c:307)
     *   ==23==    by 0x6B4F563: aligned_alloc (heap_tracker.cc:418)
     *   ==23==    by 0x7439FC6: operator new(unsigned int, std::align_val_t)
     *                           (new_opa.cc:129)
     *   ==23==    by 0x743A069: operator new[](unsigned int, std::align_val_t)
     *                           (new_opva.cc:32)
     *
     * While the aligned deallocations are bypassing the heap tracker confusing
     * valgrind:
     *
     *   ==23== Invalid free() / delete / delete[] / realloc()
     *   ==23==    at 0x4833877: free (vg_replace_malloc.c:538)
     *   ==23==    by 0x743A0C8: operator delete(void*, std::align_val_t)
     *                           (del_opa.cc:51)
     *   ==23==    by 0x743A137: operator delete[](void*, std::align_val_t)
     *                           (del_opva.cc:35
     *
     * @param size The required allocation size.
     * @param alignment The required alignment. Must be a power of 2.
     * @param[out] ptr Receives the pointer to the allocated storage.
     * @param[out] aligned_ptr Receives the pointer to the aligned buffer.
     *
     * @return True on success.
     */
    static bool alloc_buffer(const size_t size,
                             const size_t alignment,
                             char *&ptr,
                             char *&aligned_ptr)
    {
        SacAbortIf(ptr, false);
        SacAbortIf(aligned_ptr, false);

        ptr = new char[size + alignment - 1];
        SacAbortIfNot(ptr, false);

        aligned_ptr = reinterpret_cast<char *>(
            (reinterpret_cast<uintptr_t>(ptr) + alignment - 1) & -alignment);

        return true;
    }

    /**
     * Free an aligned buffer.
     *
     * @param[in, out] ptr The pointer to the allocated storage.
     * @param[in, out] aligned_ptr The pointer to the aligned buffer.
     *
     * @return True on success.
     */
    static void free_buffer(char *&ptr, char *&aligned_ptr)
    {
        delete[] ptr;

        ptr = nullptr;
        aligned_ptr = nullptr;
    }

    /**
     * Constructor.
     */
    AlignedBuffer::AlignedBuffer()
    {}

    /**
     * Destructor.
     */
    AlignedBuffer::~AlignedBuffer()
    {
        free_buffer(ptr, aligned_ptr);
    }

    /**
     * Transfer ownership of the managed buffer from \ref other to this object.
     *
     * @param other The object to transfer the buffer overship from.
     */
    void AlignedBuffer::assign(AlignedBuffer &&other)
    {
        free_buffer(ptr, aligned_ptr);

        ptr = other.ptr;
        aligned_ptr = other.aligned_ptr;
        length = other.length;
        total = other.total;

        other.ptr = {};
        other.aligned_ptr = {};
        other.length = {};
        other.total = {};
    }

    /**
     * Transfer ownership of the managed buffer from \ref other to this object.
     *
     * @param other The object to transfer the buffer overship from.
     */
    AlignedBuffer::AlignedBuffer(AlignedBuffer &&other)
    {
        assign(std::move(other));
    }

    /**
     * Transfer ownership of the managed buffer from \ref other to this object.
     *
     * @param other The object to transfer the buffer overship from.
     *
     * @return Reference to itself.
     */
    AlignedBuffer &AlignedBuffer::operator=(AlignedBuffer &&other)
    {
        assign(std::move(other));

        return *this;
    }

    /**
     * Ensures that the managed buffer is at least \ref size bytes large and is
     * aligned to \ref alignment bytes.
     *
     * Calling this method invalidates any pointes returned by get() previously.
     * The content of the buffer is copied to the new buffer if reallocation was
     * necessary. Newly allocated space is initialized.
     *
     * @param amount The requested length of teh buffer.
     * @param alignment The desired alignment. Must be a positive power of 2.
     *
     * @return True on success.
     */
    bool AlignedBuffer::ensure(const size_t amount, const size_t alignment)
    {
        /*
         * Alignment must be positive and a power of 2.
         */
        SacAbortIfNot((alignment != 0) &&
                          ((alignment & -alignment) == alignment),
                      false);

        /*
         * The current implementation always allocates a pre-aligned buffer to
         * keep the implementation simple.
         */
        SacAbortIf(alignment > max_alignment, false);

        /*
         * Do nothing if the buffer is already big enough.
         */
        if (total >= amount)
        {
            length = std::max(length, amount);
            return true;
        }

        /*
         * Compute the new allocation. Grow by 1.5x unless the requested amount
         * is even larger. This takes care of both cases: when the buffer grows
         * slowly small bit at a time and when it grow rapidly to some large
         * size.
         *
         * Use the growth factor of 1.5 to be more memory manager friendly. See:
         * https://github.com/facebook/folly/blob/main/folly/docs/FBVector.md
         */
        const size_t new_total =
            std::max(std::max(total + total / 2, initial_total), amount);

        char *new_ptr = nullptr;
        char *new_aligned_ptr = nullptr;
        SacAbortIfNot(
            alloc_buffer(new_total, max_alignment, new_ptr, new_aligned_ptr),
            false);

        /*
         * Copy the contents to the new buffer and initialize the newly
         * allocated space.
         */
        if (aligned_ptr)
        {
            memcpy(new_aligned_ptr, aligned_ptr, length);
        }
        memset(new_aligned_ptr + length, 0, new_total - length);

        free_buffer(ptr, aligned_ptr);

        ptr = new_ptr;
        aligned_ptr = new_aligned_ptr;
        length = amount;
        total = new_total;

        return true;
    }

    /**
     * @return A copy of the managed buffer.
     */
    AlignedBuffer AlignedBuffer::clone() const
    {
        AlignedBuffer result;

        if (aligned_ptr)
        {
            char *new_ptr = nullptr;
            char *new_aligned_ptr = nullptr;
            SacAbortIfNot(
                alloc_buffer(length, max_alignment, new_ptr, new_aligned_ptr),
                result);

            memcpy(new_aligned_ptr, aligned_ptr, length);
            result.ptr = new_ptr;
            result.aligned_ptr = new_aligned_ptr;
            result.length = length;
            result.total = length;
        }

        return result;
    }

} /* end namespace Drone */
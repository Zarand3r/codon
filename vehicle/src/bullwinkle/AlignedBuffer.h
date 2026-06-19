/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef ALIGNED_BUFFER_H
#define ALIGNED_BUFFER_H

#include "src/bullwinkle/all/core/util.h"

#include <cstddef>

namespace Drone
{
    namespace UnitTestUtilities
    {
        class SlateRecorder;
    }

    /**
     * This class manages an aligned memory buffer. The buffer can grow or be
     * reallocated with new alignment.
     */
    class AlignedBuffer
    {
    public:
        /**
         * The maximum aligment supported by AlignedBuffer.
         */
        static constexpr size_t max_alignment = 32;
        static_assert(max_alignment >= alignof(std::max_align_t));

        AlignedBuffer();
        ~AlignedBuffer();

        AlignedBuffer(AlignedBuffer &&other);
        AlignedBuffer &operator=(AlignedBuffer &&other);

        bool ensure(const size_t amount, const size_t alignment);

        AlignedBuffer clone() const;

        /**
         * @return The managed buffer pointer.
         */
        char *data()
        {
            return aligned_ptr;
        }

        /**
         * @return The managed buffer pointer.
         */
        const char *data() const
        {
            return aligned_ptr;
        }

        /**
         * @return The amount of the buffer allocated so far.
         */
        size_t size() const
        {
            return length;
        }

        /**
         * @return The capacity of the managed buffer.
         */
        size_t capacity() const
        {
            return total;
        }

        /**
         * @return True if the buffer is allocated.
         */
        operator bool() const
        {
            return data() != nullptr;
        }

    private:
        SX_DISALLOW_COPY_AND_ASSIGN(AlignedBuffer);

        void assign(AlignedBuffer &&other);

        /*
         * Allow SlateRecorder to manufacture a custom AlignedBuffer.
         */
        friend class UnitTestUtilities::SlateRecorder;

        /**
         * The pointer to the allocated buffer.
         */
        char *ptr {};

        /**
         * The aligned pointer to the managed buffer.
         */
        char *aligned_ptr {};

        /**
         * The amount of space requested so far.
         */
        size_t length {};

        /**
         * The total size of the allocated buffer.
         */
        size_t total {};
    };

} /* end namespace Drone */

#endif /* ALIGNED_BUFFER_H */
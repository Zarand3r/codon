/// @file drone/sat/util/fast_sort/fast_sort.cc
#include "drone/sat/util/fast_sort/fast_sort.h"
namespace Drone::Sat
{
    /// Internal core sort of n elements of size kBytes.
    template <U64 kBytes>
    ALWAYS_INLINE static void SortInternal(void *__restrict pScratch,
                                           void *__restrict p,
                                           const U64 n) noexcept
    {
        for (U32 iByte = 0; iByte < kBytes; ++iByte)
        {
            const U8 *__restrict const pA =
                reinterpret_cast<const U8 *>(iByte & 1 ? pScratch : p);
            U8 *__restrict const pB =
                reinterpret_cast<U8 *>(iByte & 1 ? p : pScratch);
            alignas(32) U32 byteCounts[256]{};
            for (U64 i = 0; i < n * kBytes; i += kBytes)
                byteCounts[pA[i + iByte]] += kBytes;
            for (U32 i = 0, sum = 0; i < 256; ++i)
            {
                const U32 tmp = byteCounts[i];
                byteCounts[i] = sum;
                sum += tmp;
            }
            for (U64 i = 0; i < n * kBytes; i += kBytes)
            {
                __builtin_memcpy(pB + byteCounts[pA[i + iByte]], pA + i,
                                 kBytes);
                byteCounts[pA[i + iByte]] += kBytes;
            }
        }
        if constexpr (kBytes & 1)
        {
            __builtin_memcpy(p, pScratch, n);
        }
    }
    void Sort(U8 *__restrict pScratch, U8 *__restrict p, const U64 n) noexcept
    {
        SortInternal<sizeof(U8)>(pScratch, p, n);
    }
    void Sort(I8 *__restrict pScratch, I8 *__restrict p, const U64 n) noexcept
    {
        for (U64 i = 0; i < n; ++i)
            p[i] ^= 0x80;
        SortInternal<sizeof(I8)>(pScratch, p, n);
        for (U64 i = 0; i < n; ++i)
            p[i] ^= 0x80;
    }
    void Sort(U16 *__restrict pScratch, U16 *__restrict p, const U64 n) noexcept
    {
        SortInternal<sizeof(U16)>(pScratch, p, n);
    }
    void Sort(I16 *__restrict pScratch, I16 *__restrict p, const U64 n) noexcept
    {
        for (U64 i = 0; i < n; ++i)
            p[i] ^= 0x8000;
        SortInternal<sizeof(I16)>(pScratch, p, n);
        for (U64 i = 0; i < n; ++i)
            p[i] ^= 0x8000;
    }
    void Sort(U32 *__restrict pScratch, U32 *__restrict p, const U64 n) noexcept
    {
        SortInternal<sizeof(U32)>(pScratch, p, n);
    }
    void Sort(I32 *__restrict pScratch, I32 *__restrict p, const U64 n) noexcept
    {
        for (U64 i = 0; i < n; ++i)
            p[i] ^= 0x80000000;
        SortInternal<sizeof(I32)>(pScratch, p, n);
        for (U64 i = 0; i < n; ++i)
            p[i] ^= 0x80000000;
    }
    void Sort(U64 *__restrict pScratch, U64 *__restrict p, const U64 n) noexcept
    {
        SortInternal<sizeof(U64)>(pScratch, p, n);
    }
    void Sort(I64 *__restrict pScratch, I64 *__restrict p, const U64 n) noexcept
    {
        for (U64 i = 0; i < n; ++i)
            p[i] ^= 0x8000000000000000;
        SortInternal<sizeof(I64)>(pScratch, p, n);
        for (U64 i = 0; i < n; ++i)
            p[i] ^= 0x8000000000000000;
    }
#ifdef __SIZEOF_INT128__
    void Sort(U128 *__restrict pScratch, U128 *__restrict p,
              const U64 n) noexcept
    {
        SortInternal<sizeof(U128)>(pScratch, p, n);
    }
    void Sort(I128 *__restrict pScratch, I128 *__restrict p,
              const U64 n) noexcept
    {
        for (U64 i = 0; i < n; ++i)
            p[i] ^= U128(1) << 127;
        SortInternal<sizeof(I128)>(pScratch, p, n);
        for (U64 i = 0; i < n; ++i)
            p[i] ^= U128(1) << 127;
    }
#endif
    void Sort(float *__restrict pScratch, float *__restrict p,
              const U64 n) noexcept
    {
        for (U64 i = 0; i < n; ++i)
            p[i] = U32BitsAsFloat((FloatBitsAsU32(p[i]) | (1U << 31)) ^
                                  U32(I32(FloatBitsAsU32(p[i])) >> 31));
        SortInternal<sizeof(float)>(pScratch, p, n);
        for (U64 i = 0; i < n; ++i)
            p[i] = U32BitsAsFloat((FloatBitsAsU32(p[i]) & ~(1U << 31)) ^
                                  U32(I32(~FloatBitsAsU32(p[i])) >> 31));
    }
} // namespace Drone::Sat
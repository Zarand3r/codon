/// @file drone/sat/util/fast_sort/fast_sort.h
#pragma once
#include "drone/sat/util/perf_util/perf_util.h"
namespace Drone::Sat
{
    /// Sort array of n elements at 'p'. Supply a buffer of equal or greater
    /// size in 'pScratch' for use as a scratch buffer by the implementation.
    /// Its contents prior to the call do not matter; its contents after the
    /// call are undefined.
    /// @{
    void Sort(U8 *__restrict pScratch, U8 *__restrict p, const U64 n) noexcept;
    void Sort(I8 *__restrict pScratch, I8 *__restrict p, const U64 n) noexcept;
    void Sort(U16 *__restrict pScratch, U16 *__restrict p,
              const U64 n) noexcept;
    void Sort(I16 *__restrict pScratch, I16 *__restrict p,
              const U64 n) noexcept;
    void Sort(U32 *__restrict pScratch, U32 *__restrict p,
              const U64 n) noexcept;
    void Sort(I32 *__restrict pScratch, I32 *__restrict p,
              const U64 n) noexcept;
    void Sort(U64 *__restrict pScratch, U64 *__restrict p,
              const U64 n) noexcept;
    void Sort(I64 *__restrict pScratch, I64 *__restrict p,
              const U64 n) noexcept;
#ifdef __SIZEOF_INT128__
    void Sort(U128 *__restrict pScratch, U128 *__restrict p,
              const U64 n) noexcept;
    void Sort(I128 *__restrict pScratch, I128 *__restrict p,
              const U64 n) noexcept;
#endif
    void Sort(float *__restrict pScratch, float *__restrict p,
              const U64 n) noexcept;
    /// @}
} // namespace Drone::Sat
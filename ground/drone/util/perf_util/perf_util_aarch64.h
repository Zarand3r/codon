/// @file src/utils/perf_util/perf_util_aarch64.h
#pragma once
#ifdef __aarch64__
#include "src/utils/perf_util/perf_util_core.h"
#include <arm_neon.h>
namespace Drone::Perf
{
    /// Fused multiply-add. Computes c + a * b with no rounding between the two
    /// operations, and in a single instruction. Note that the argument order
    /// differs from x86, where it's a, b, c; in AArch64 it's c, a, b.
    ///
    /// A longstanding gcc bug that's STILL not fixed means the intrinsic
    /// (vmlaq_f32) sometimes violates the architectural standard and generates
    /// a separate mul + add, which is not just bad for performance but produces
    /// a different result due to the rounding, so, use this instead.
    static inline float32x4_t Vmla(float32x4_t c, const float32x4_t a,
                                   const float32x4_t b)
    {
        asm("fmla %0.4s, %1.4s, %2.4s" : "+w"(c) : "w"(a), "w"(b));
        return c;
    }
    /// Fused negate-multiply-add. Computes c - a * b with no rounding between
    /// the two operations, and in a single instruction. Note that the argument
    /// order differs from x86, where it's a, b, c; in AArch64 it's c, a, b.
    ///
    /// A longstanding gcc bug that's STILL not fixed means the intrinsic
    /// (vmlsq_f32) sometimes violates the architectural standard and generates
    /// a separate mul + add, which is not just bad for performance but produces
    /// a different result due to the rounding, so, use this instead.
    static inline float32x4_t Vmls(float32x4_t c, const float32x4_t a,
                                   const float32x4_t b)
    {
        asm("fmls %0.4s, %1.4s, %2.4s" : "+w"(c) : "w"(a), "w"(b));
        return c;
    }
    /// Dot two v.4s, splatting (broadcasting) the scalar result to each of the
    /// 4 elements.
    ///
    /// x y z 0
    /// ->
    /// d d d d
    ///
    /// PRECONDITION: w-components are 0.
    static inline float32x4_t DotSplat(float32x4_t a, float32x4_t b)
    {
        a = vmulq_f32(a, b);
        a = vpaddq_f32(a, a);
        a = vpaddq_f32(a, a);
        return a;
    }
    /// Dot two v.4s, producing the scalar result ONLY in the 0th element.
    ///
    /// x y z -
    /// ->
    /// d - - -
    static inline float32x4_t DotSingle(float32x4_t a, float32x4_t b)
    {
        return DotSplat(a, b);
    }
    /// Extract the MSBs of each of the 16 bytes in 'a' into a contiguous block
    /// of 16 bits for return.
    static inline U32 Movemask(uint8x16_t a)
    {
        a = vshrq_n_u8(a, 7);
        a = vreinterpretq_u8_u16(
            vsraq_n_u16(vreinterpretq_u16_u8(a), vreinterpretq_u16_u8(a), 7));
        a = vreinterpretq_u8_u32(
            vsraq_n_u32(vreinterpretq_u32_u8(a), vreinterpretq_u32_u8(a), 14));
        a = vreinterpretq_u8_u64(
            vsraq_n_u64(vreinterpretq_u64_u8(a), vreinterpretq_u64_u8(a), 28));
        const U8 b = vgetq_lane_u8(a, 0);
        const U8 c = vgetq_lane_u8(a, 8);
        return (U32(c) << 8) | b;
    }
} // namespace Drone::Perf
#endif // __aarch64__
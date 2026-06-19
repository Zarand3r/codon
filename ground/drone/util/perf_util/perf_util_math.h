/// @file src/utils/perf_util/perf_util_math.h
#pragma once
#include "src/utils/perf_util/perf_util_avx2.h"
#include "src/utils/perf_util/perf_util_core.h"
namespace Drone::Perf
{
    //! @cond Doxygen_Suppress
    static constexpr float kPosInf = __builtin_inff();
    static constexpr float kNegInf = -kPosInf;
    static constexpr float kNan = __builtin_nanf("");
    static constexpr float kNegNan = __builtin_copysignf(kNan, -0.0f);
    static constexpr float kFloatMax = __FLT_MAX__;
    static constexpr float kNegFloatMax = -kFloatMax;
    /// Smallest representable positive normal float. Denormals should be off
    /// anyway.
    static constexpr float kFloatMin = __FLT_MIN__;
    /// Smallest representable negative normal float. Denormals should be off
    /// anyway.
    static constexpr float kNegFloatMin = -kFloatMin;
    static constexpr float kPi = 3.14159265f;
    static constexpr float kTwoPi = 6.2831853f;
    static constexpr float kPiOverTwo = 1.57079632679f;
    static constexpr float kThreePiOverTwo = 4.71238898f;
    // clang-format off
static constexpr I8   Abs(I8   x) { return x < 0 ? -x : x; }
static constexpr I16  Abs(I16  x) { return x < 0 ? -x : x; }
static constexpr I32  Abs(I32  x) { return x < 0 ? -x : x; }
static constexpr I64  Abs(I64  x) { return x < 0 ? -x : x; }
#ifdef __SIZEOF_INT128__
static constexpr I128 Abs(I128 x) { return x < 0 ? -x : x; }
#endif
static constexpr I8   Max(I8   a, I8   b) { return a > b ? a : b; }
static constexpr U8   Max(U8   a, U8   b) { return a > b ? a : b; }
static constexpr I16  Max(I16  a, I16  b) { return a > b ? a : b; }
static constexpr U16  Max(U16  a, U16  b) { return a > b ? a : b; }
static constexpr I32  Max(I32  a, I32  b) { return a > b ? a : b; }
static constexpr U32  Max(U32  a, U32  b) { return a > b ? a : b; }
static constexpr I64  Max(I64  a, I64  b) { return a > b ? a : b; }
static constexpr U64  Max(U64  a, U64  b) { return a > b ? a : b; }
#ifdef __SIZEOF_INT128__
static constexpr I128 Max(I128 a, I128 b) { return a > b ? a : b; }
static constexpr U128 Max(U128 a, U128 b) { return a > b ? a : b; }
#endif
static constexpr I8   Min(I8   a, I8   b) { return a < b ? a : b; }
static constexpr U8   Min(U8   a, U8   b) { return a < b ? a : b; }
static constexpr I16  Min(I16  a, I16  b) { return a < b ? a : b; }
static constexpr U16  Min(U16  a, U16  b) { return a < b ? a : b; }
static constexpr I32  Min(I32  a, I32  b) { return a < b ? a : b; }
static constexpr U32  Min(U32  a, U32  b) { return a < b ? a : b; }
static constexpr I64  Min(I64  a, I64  b) { return a < b ? a : b; }
static constexpr U64  Min(U64  a, U64  b) { return a < b ? a : b; }
#ifdef __SIZEOF_INT128__
static constexpr I128 Min(I128 a, I128 b) { return a < b ? a : b; }
static constexpr U128 Min(U128 a, U128 b) { return a < b ? a : b; }
#endif
static inline U32 FloatBitsAsU32(float x)
{
    static_assert(sizeof(float) == 4);
    static_assert(sizeof(U32) == 4);
    U32 r;
    __builtin_memcpy(&r, &x, 4);
    return r;
}
static inline float U32BitsAsFloat(U32 x)
{
    static_assert(sizeof(float) == 4);
    static_assert(sizeof(U32) == 4);
    float r;
    __builtin_memcpy(&r, &x, 4);
    return r;
}
static inline float Min(float a, float b)
{
    return a < b ? a : b;
}
static inline double Min(double a, double b)
{
    return a < b ? a : b;
}
static inline float Max(float a, float b)
{
    return a > b ? a : b;
}
static inline double Max(double a, double b)
{
    return a > b ? a : b;
}
#ifndef __AVX2__
static inline float Sqrt(float x)
{
    return __builtin_sqrtf(x);
}
static inline float RoundToNearest(float x)
{
    // Requires the current rounding mode to be NEAREST (which is the default).
    return __builtin_rintf(x);
}
static inline float RoundToNegInf(float x)
{
    return __builtin_floorf(x);
}
static inline float RoundToPosInf(float x)
{
    return __builtin_ceilf(x);
}
static inline float RoundToZero(float x)
{
    return __builtin_truncf(x);
}
static inline I32 RoundToNearestAndConvert(float x)
{
    // Requires the current rounding mode to be NEAREST (which is the default).
    return I32(__builtin_lrintf(x));
}
/// Compute a * b + c without allowing the compiler to automatically ffp-contract the expression
/// into an FMA (fused multiply-add), which performs no rounding between the multiplication and
/// addition steps, resulting in slightly different floating-point output.
///
/// Useful if you need to enforce numerically exact results between e.g. debug and release builds,
/// or to match a vector or assembly implementation that intentionally doesn't do an FMA for some
/// reason.
static inline float SuppressFma(float a, float b, float c)
{
    const volatile float t1 = a * b;
    const volatile float t2 = c;
    return t1 + t2;
}
/// Force the complete evaluation of an expression yielding a float and prevent the compiler from
/// making any further assumptions about the value of that expression as returned from ForceEval().
static inline float ForceEval(float x)
{
    const volatile float t1 = x;
    return t1;
}
#endif //__AVX2__
static constexpr I8   Clamp(I8   x, I8   lo, I8   hi) { return Min(Max(x, lo), hi); }
static constexpr U8   Clamp(U8   x, U8   lo, U8   hi) { return Min(Max(x, lo), hi); }
static constexpr I16  Clamp(I16  x, I16  lo, I16  hi) { return Min(Max(x, lo), hi); }
static constexpr U16  Clamp(U16  x, U16  lo, U16  hi) { return Min(Max(x, lo), hi); }
static constexpr I32  Clamp(I32  x, I32  lo, I32  hi) { return Min(Max(x, lo), hi); }
static constexpr U32  Clamp(U32  x, U32  lo, U32  hi) { return Min(Max(x, lo), hi); }
static constexpr I64  Clamp(I64  x, I64  lo, I64  hi) { return Min(Max(x, lo), hi); }
static constexpr U64  Clamp(U64  x, U64  lo, U64  hi) { return Min(Max(x, lo), hi); }
#ifdef __SIZEOF_INT128__
static constexpr I128 Clamp(I128 x, I128 lo, I128 hi) { return Min(Max(x, lo), hi); }
static constexpr U128 Clamp(U128 x, U128 lo, U128 hi) { return Min(Max(x, lo), hi); }
#endif
static inline float Clamp(float x, float lo, float hi) { return Min(Max(x, lo), hi); }
static inline double Clamp(double x, double lo, double hi) { return Min(Max(x, lo), hi); }
    // clang-format on
    static inline float Round(float x) { return RoundToNearest(x); }
    static inline float Floor(float x) { return RoundToNegInf(x); }
    static inline float Ceil(float x) { return RoundToPosInf(x); }
    static inline float Trunc(float x) { return RoundToZero(x); }
    static constexpr float Sqr(float x) { return x * x; }
    static constexpr float Abs(float x) { return __builtin_fabsf(x); }
    static constexpr float Copysign(float dst, float signFrom)
    {
        return __builtin_copysignf(dst, signFrom);
    }
    static inline float Fma(float a, float b, float c)
    {
        return __builtin_fmaf(a, b, c);
    }
    static constexpr float Deg2Rad(float deg)
    {
        return deg * float(3.141592653589793 / 180.0);
    }
    static constexpr float Rad2Deg(float rad)
    {
        return rad * float(180.0 / 3.141592653589793);
    }
    //! @endcond
} // namespace Drone::Perf
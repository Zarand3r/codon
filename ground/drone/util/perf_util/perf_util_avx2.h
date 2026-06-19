/// @file src/utils/perf_util/perf_util_avx2.h
#pragma once
#ifdef __AVX2__
#include "src/utils/perf_util/perf_util_core.h"
#include <immintrin.h>
namespace Drone::Perf
{
    static inline float Sqrt(float x)
    {
        return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ss(x)));
    }
    static inline float RoundToNearest(float x)
    {
        return _mm_cvtss_f32(
            _mm_round_ss(_mm_set_ss(x), _mm_set_ss(x),
                         _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
    }
    static inline float RoundToNegInf(float x)
    {
        return _mm_cvtss_f32(
            _mm_round_ss(_mm_set_ss(x), _mm_set_ss(x),
                         _MM_FROUND_TO_NEG_INF | _MM_FROUND_NO_EXC));
    }
    static inline float RoundToPosInf(float x)
    {
        return _mm_cvtss_f32(
            _mm_round_ss(_mm_set_ss(x), _mm_set_ss(x),
                         _MM_FROUND_TO_POS_INF | _MM_FROUND_NO_EXC));
    }
    static inline float RoundToZero(float x)
    {
        return _mm_cvtss_f32(
            _mm_round_ss(_mm_set_ss(x), _mm_set_ss(x),
                         _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC));
    }
    static inline I32 RoundToNearestAndConvert(float x)
    {
        // Requires the current rounding mode to be NEAREST (which is the
        // default).
        return _mm_cvtss_si32(_mm_set_ss(x));
    }
    /// x86 CPU Vendor.
    enum class CpuVendor : U32
    {
        kAmd,
        kIntel,
        kUnknown,
    };
    /// Get an x86 CPU's vendor.
    static inline CpuVendor GetCpuVendor()
    {
        // The CPUID instruction is slow. Intel and AMD implement the
        // [v]rcp[p|s]s (approx reciprocal) instruction differently, which we
        // can use to determine the vendor cheaply.
        switch (U32(
            _mm_cvtsi128_si32(_mm_castps_si128(_mm_rcp_ss(_mm_set_ss(1e-9f))))))
        {
        case 0x4e6e6800:
            return CpuVendor::kAmd;
        case 0x4e6e6000:
            return CpuVendor::kIntel;
        default:
            return CpuVendor::kUnknown;
        }
    }
    /// Compute a * b + c without allowing the compiler to automatically
    /// ffp-contract the expression into an FMA (fused multiply-add), which
    /// performs no rounding between the multiplication and addition steps,
    /// resulting in slightly different floating-point output.
    ///
    /// Useful if you need to enforce numerically exact results between e.g.
    /// debug and release builds, or to match a vector or assembly
    /// implementation that intentionally doesn't do an FMA for some reason.
    static inline float SuppressFma(float a, float b, float c)
    {
        a *= b;
        asm("" : "+x"(a), "+x"(c));
        return a + c;
    }
    /// Force the complete evaluation of an expression yielding a float and
    /// prevent the compiler from making any further assumptions about the value
    /// of that expression as returned from ForceEval().
    static inline float ForceEval(float x)
    {
        asm("" : "+x"(x));
        return x;
    }
    /// Compute element-wise absolute value.
    static inline __m128 Abs(const __m128 v)
    {
        return _mm_and_ps(v, _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF)));
    }
    /// Compute element-wise absolute value.
    static inline __m256 Abs(const __m256 v)
    {
        return _mm256_and_ps(
            v, _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF)));
    }
    /// Dot two __m128s, splatting (broadcasting) the scalar result to each of
    /// the 4 elements.
    ///
    /// x y z 0
    /// ->
    /// d d d d
    ///
    /// PRECONDITION: w-components are 0.
    static inline __m128 DotSplat(__m128 a, __m128 b) noexcept
    {
        a = _mm_mul_ps(a, b);
        a = _mm_add_ps(a, _mm_shuffle_ps(a, a, 177));
        a = _mm_add_ps(a, _mm_shuffle_ps(a, a, 78));
        return a;
    }
    /// Dot two __m256s, dotting the low lanes and high lanes independently,
    /// splatting (broadcasting) the scalar result to each of the 4 elements of
    /// the lane.
    ///
    /// x1 xy z1  0 | x2 y2 z2  0
    /// ->
    /// d1 d1 d1 d1 | d2 d2 d2 d2
    ///
    /// PRECONDITION: w-components are 0.
    static inline __m256 DotSplat(__m256 a, __m256 b) noexcept
    {
        a = _mm256_mul_ps(a, b);
        a = _mm256_add_ps(a, _mm256_shuffle_ps(a, a, 177));
        a = _mm256_add_ps(a, _mm256_shuffle_ps(a, a, 78));
        return a;
    }
    /// Dot two __m128s, producing the scalar result ONLY in the 0th element.
    ///
    /// x y z -
    /// ->
    /// d - - -
    static inline __m128 DotSingle(__m128 a, __m128 b) noexcept
    {
        a = _mm_mul_ps(a, b);
        return _mm_add_ps(_mm_add_ps(a, _mm_shuffle_ps(a, a, 177)),
                          _mm_shuffle_ps(a, a, 78));
    }
    /// Dot two __m256s, dotting the low lanes and high lanes independently,
    /// producing the two scalar results (one for each lane) ONLY in the low
    /// element of each lane, i.e. elements 0 and 4.
    ///
    //  x1 xy z1  - | x2 y2 z2  -
    /// ->
    /// d1  -  -  - | d2  -  -  -
    static inline __m256 DotSingle(__m256 a, __m256 b) noexcept
    {
        a = _mm256_mul_ps(a, b);
        return _mm256_add_ps(_mm256_add_ps(a, _mm256_shuffle_ps(a, a, 177)),
                             _mm256_shuffle_ps(a, a, 78));
    }
    /// Compute element-wise natural log.
    ///
    /// Subnormal inputs will be treated as zero and therefore return -inf.
    ///
    /// In vecmath_test.cc we check all 2^32 float inputs against the standard
    /// library implementation, requiring worst-case relative error < 2.3e-7.
    ///
    /// The basic approach is as follows:
    /// Normal floats are represented as a normalized mantissa in [1, 2), and an
    /// exponent, e.g. m*2^e. To make the discussion easier, let's do
    /// log2 instead of ln since they're simply related by a proportionality
    /// constant at the end. log2(m*2^e) == log2(m) + e This is nice because m
    /// is bounded to the domain [1, 2), so log2(m) can be well-approximated by
    /// a Taylor or Pade approximation. We use Taylor, add the exponent, and
    /// finally perform branchless fixup of special case inputs (since they must
    /// be handled independently per element of the vector), of which there are
    /// 3:
    /// - negatives must go to NaN
    /// - zero/denormal must go to -inf
    /// - NaN/+inf must remain unchanged
    static inline __m128 Log(__m128 v)
    {
        // Separate mantissa and exponent.
        const __m128i m =
            _mm_and_si128(_mm_castps_si128(v), _mm_set1_epi32(0x007FFFFF));
        const __m128i k = _mm_cmpgt_epi32(m, _mm_set1_epi32(3474675));
        const __m128 e = _mm_cvtepi32_ps(
            _mm_sub_epi32(_mm_srli_epi32(_mm_castps_si128(v), 23),
                          _mm_add_epi32(k, _mm_set1_epi32(127))));
        __m128 x =
            _mm_castsi128_ps(_mm_or_si128(m, _mm_set1_epi32(0x3F000000)));
        x = _mm_add_ps(x, _mm_andnot_ps(_mm_castsi128_ps(k), x));
        x = _mm_sub_ps(x, _mm_set1_ps(1.0f));
        // Compute Taylor approximation of log(m).
        const __m128 x2 = _mm_mul_ps(x, x);
        const __m128 x4 = _mm_mul_ps(x2, x2);
        const __m128 x8 = _mm_mul_ps(x4, x4);
        __m128 r = _mm_fmadd_ps(
            _mm_fmadd_ps(_mm_fmadd_ps(_mm_set1_ps(-0.1151461f), x,
                                      _mm_set1_ps(0.116769987f)),
                         x2,
                         _mm_fmadd_ps(_mm_set1_ps(-0.124201408f), x,
                                      _mm_set1_ps(0.14249322787f))),
            x4,
            _mm_fmadd_ps(
                _mm_fmadd_ps(_mm_set1_ps(-0.166680576f), x,
                             _mm_set1_ps(0.20000714765f)),
                x2,
                _mm_fmadd_ps(_mm_set1_ps(7.0376836292e-2f), x8,
                             _mm_fmadd_ps(_mm_set1_ps(-0.24999994f), x,
                                          _mm_set1_ps(0.3333333117f)))));
        // Add exponent.
        r = _mm_mul_ps(r, _mm_mul_ps(x2, x));
        r = _mm_fmadd_ps(e, _mm_set1_ps(-2.1219444e-4f), r);
        r = _mm_add_ps(r, _mm_fmadd_ps(x2, _mm_set1_ps(-0.5f), x));
        r = _mm_fmadd_ps(e, _mm_set1_ps(0.693359375f), r);
        // Handle special cases.
        r = _mm_blendv_ps(
            r, _mm_castsi128_ps(_mm_set1_epi32(0xFF800000)),
            _mm_cmp_ps(v, _mm_set1_ps(1.17549435e-38f), _CMP_LT_OQ));
        r = _mm_blendv_ps(
            r, v,
            _mm_cmp_ps(v, _mm_castsi128_ps(_mm_set1_epi32(0x7F800000)),
                       _CMP_NLT_UQ));
        return _mm_blendv_ps(r, _mm_castsi128_ps(_mm_set1_epi32(0xFFC00000)),
                             _mm_cmp_ps(v, _mm_setzero_ps(), _CMP_LT_OQ));
    }
    /// Compute element-wise natural log.
    ///
    /// Subnormal inputs will be treated as zero and therefore return -inf.
    ///
    /// See __m128 overload for more docs.
    static inline __m256 Log(__m256 v)
    {
        // Separate mantissa and exponent.
        const __m256i m = _mm256_and_si256(_mm256_castps_si256(v),
                                           _mm256_set1_epi32(0x007FFFFF));
        const __m256i k = _mm256_cmpgt_epi32(m, _mm256_set1_epi32(3474675));
        const __m256 e = _mm256_cvtepi32_ps(
            _mm256_sub_epi32(_mm256_srli_epi32(_mm256_castps_si256(v), 23),
                             _mm256_add_epi32(k, _mm256_set1_epi32(127))));
        __m256 x = _mm256_castsi256_ps(
            _mm256_or_si256(m, _mm256_set1_epi32(0x3F000000)));
        x = _mm256_add_ps(x, _mm256_andnot_ps(_mm256_castsi256_ps(k), x));
        x = _mm256_sub_ps(x, _mm256_set1_ps(1.0f));
        // Compute Taylor approximation of log(m).
        const __m256 x2 = _mm256_mul_ps(x, x);
        const __m256 x4 = _mm256_mul_ps(x2, x2);
        const __m256 x8 = _mm256_mul_ps(x4, x4);
        __m256 r = _mm256_fmadd_ps(
            _mm256_fmadd_ps(_mm256_fmadd_ps(_mm256_set1_ps(-0.1151461f), x,
                                            _mm256_set1_ps(0.116769987f)),
                            x2,
                            _mm256_fmadd_ps(_mm256_set1_ps(-0.124201408f), x,
                                            _mm256_set1_ps(0.14249322787f))),
            x4,
            _mm256_fmadd_ps(
                _mm256_fmadd_ps(_mm256_set1_ps(-0.166680576f), x,
                                _mm256_set1_ps(0.20000714765f)),
                x2,
                _mm256_fmadd_ps(
                    _mm256_set1_ps(7.0376836292e-2f), x8,
                    _mm256_fmadd_ps(_mm256_set1_ps(-0.24999994f), x,
                                    _mm256_set1_ps(0.3333333117f)))));
        // Add exponent.
        r = _mm256_mul_ps(r, _mm256_mul_ps(x2, x));
        r = _mm256_fmadd_ps(e, _mm256_set1_ps(-2.1219444e-4f), r);
        r = _mm256_add_ps(r, _mm256_fmadd_ps(x2, _mm256_set1_ps(-0.5f), x));
        r = _mm256_fmadd_ps(e, _mm256_set1_ps(0.693359375f), r);
        // Handle special cases.
        r = _mm256_blendv_ps(
            r, _mm256_castsi256_ps(_mm256_set1_epi32(0xFF800000)),
            _mm256_cmp_ps(v, _mm256_set1_ps(1.17549435e-38f), _CMP_LT_OQ));
        r = _mm256_blendv_ps(
            r, v,
            _mm256_cmp_ps(v, _mm256_castsi256_ps(_mm256_set1_epi32(0x7F800000)),
                          _CMP_NLT_UQ));
        return _mm256_blendv_ps(
            r, _mm256_castsi256_ps(_mm256_set1_epi32(0xFFC00000)),
            _mm256_cmp_ps(v, _mm256_setzero_ps(), _CMP_LT_OQ));
    }
    static inline __m128 Log10(__m128 v)
    {
        // 1/log(10)
        return _mm_mul_ps(Log(v), _mm_set1_ps(0.4342944819f));
    }
    static inline __m256 Log10(__m256 v)
    {
        // 1/log(10)
        return _mm256_mul_ps(Log(v), _mm256_set1_ps(0.4342944819f));
    }
    /// Compute element-wise exponentiation, e^x.
    ///
    /// In vecmath_test.cc we check all 2^32 float inputs against the standard
    /// library implementation, requiring worst-case relative error < 2.3e-7.
    ///
    /// The basic approach is as follows:
    /// We divide our input, x, by log(2), breaking it into integer and
    /// fractional components: x == a*log(2) + b where a is an integer Now we
    /// exp(): exp(a*log(2) + b) == exp(a*log(2)) * exp(b) == 2^a + exp(b) 2^a
    /// with integer a is easy, and b is bounded to [-0.5, 0.5] so exp(b) can be
    /// well-approximated by a Taylor or Pade approximation. We use Taylor,
    /// compute and add 2^a, and finally perform branchless fixup of special
    /// case inputs (since they must be handled independently per element of the
    /// vector), of which there are 3:
    /// - inputs that are too large must go to +inf
    /// - inputs that are too small must go to 0.0f
    /// - NaN must remain unchanged
    static inline __m128 Exp(__m128 v)
    {
        // Divide input by log(2) and break into integer and fractional
        // components.
        const __m128 r =
            _mm_round_ps(_mm_mul_ps(v, _mm_set1_ps(1.44269504f)), 8);
        __m128 m = _mm_fmadd_ps(r, _mm_set1_ps(-0.693359375f), v);
        m = _mm_fmadd_ps(r, _mm_set1_ps(2.1219444e-4f), m);
        // Compute Taylor approximation of exp(b).
        const __m128 x2 = _mm_mul_ps(m, m);
        const __m128 x4 = _mm_mul_ps(x2, x2);
        const __m128 e = _mm_min_ps(
            _mm_castsi128_ps(_mm_slli_epi32(
                _mm_castps_si128(_mm_add_ps(r, _mm_set1_ps(8388735.0f))), 23)),
            _mm_set1_ps(3.40282346639e38f));
        __m128 z = _mm_fmadd_ps(
            _mm_fmadd_ps(
                _mm_fmadd_ps(
                    _mm_fmadd_ps(_mm_set1_ps(1.0f / 120.0f), m,
                                 _mm_set1_ps(1.0f / 24.0f)),
                    x2,
                    _mm_fmadd_ps(_mm_fmadd_ps(_mm_set1_ps(1.0f / 5040.0f), m,
                                              _mm_set1_ps(1.0f / 720.0f)),
                                 x4,
                                 _mm_fmadd_ps(_mm_set1_ps(1.0f / 6.0f), m,
                                              _mm_set1_ps(1.0f / 2.0f)))),
                x2, m),
            e, e);
        // Handle special cases.
        z = _mm_blendv_ps(z, _mm_castsi128_ps(_mm_set1_epi32(0x7F800000)),
                          _mm_cmp_ps(v, _mm_set1_ps(88.72283936f), _CMP_GE_OQ));
        return _mm_andnot_ps(
            _mm_cmp_ps(v, _mm_set1_ps(-88.37625885f), _CMP_LE_OQ), z);
    }
    /// Compute element-wise exponentiation, e^x.
    ///
    /// See __m128 overload for more docs.
    static inline __m256 Exp(__m256 v)
    {
        // Divide input by log(2) and break into integer and fractional
        // components.
        const __m256 r =
            _mm256_round_ps(_mm256_mul_ps(v, _mm256_set1_ps(1.44269504f)), 8);
        __m256 m = _mm256_fmadd_ps(r, _mm256_set1_ps(-0.693359375f), v);
        m = _mm256_fmadd_ps(r, _mm256_set1_ps(2.1219444e-4f), m);
        // Compute Taylor approximation of exp(b).
        const __m256 x2 = _mm256_mul_ps(m, m);
        const __m256 x4 = _mm256_mul_ps(x2, x2);
        const __m256 e = _mm256_min_ps(_mm256_castsi256_ps(_mm256_slli_epi32(
                                           _mm256_castps_si256(_mm256_add_ps(
                                               r, _mm256_set1_ps(8388735.0f))),
                                           23)),
                                       _mm256_set1_ps(3.40282346639e38f));
        __m256 z = _mm256_fmadd_ps(
            _mm256_fmadd_ps(
                _mm256_fmadd_ps(
                    _mm256_fmadd_ps(_mm256_set1_ps(1.0f / 120.0f), m,
                                    _mm256_set1_ps(1.0f / 24.0f)),
                    x2,
                    _mm256_fmadd_ps(
                        _mm256_fmadd_ps(_mm256_set1_ps(1.0f / 5040.0f), m,
                                        _mm256_set1_ps(1.0f / 720.0f)),
                        x4,
                        _mm256_fmadd_ps(_mm256_set1_ps(1.0f / 6.0f), m,
                                        _mm256_set1_ps(1.0f / 2.0f)))),
                x2, m),
            e, e);
        // Handle special cases.
        z = _mm256_blendv_ps(
            z, _mm256_castsi256_ps(_mm256_set1_epi32(0x7F800000)),
            _mm256_cmp_ps(v, _mm256_set1_ps(88.72283936f), _CMP_GE_OQ));
        return _mm256_andnot_ps(
            _mm256_cmp_ps(v, _mm256_set1_ps(-88.37625885f), _CMP_LE_OQ), z);
    }
    /// Compute element-wise exponentiation, 10^x.
    ///
    /// See Exp() for more docs, as the approach is the same.
    static inline __m128 Pow10(__m128 v)
    {
        // Divide input by log10(2) and break into integer and fractional
        // components.
        const __m128 r =
            _mm_round_ps(_mm_mul_ps(v, _mm_set1_ps(3.321928024f)), 8);
        __m128 m = _mm_fmadd_ps(r, _mm_set1_ps(-0.301025391f), v);
        m = _mm_fmadd_ps(r, _mm_set1_ps(-4.60503907e-6f), m);
        m = _mm_mul_ps(m, _mm_set1_ps(2.302585125f));
        // Compute Taylor approximation of 10^b.
        const __m128 x2 = _mm_mul_ps(m, m);
        const __m128 x4 = _mm_mul_ps(x2, x2);
        const __m128 e = _mm_min_ps(
            _mm_castsi128_ps(_mm_slli_epi32(
                _mm_castps_si128(_mm_add_ps(r, _mm_set1_ps(8388735.0f))), 23)),
            _mm_set1_ps(3.40282346639e38f));
        __m128 z = _mm_fmadd_ps(
            _mm_fmadd_ps(
                _mm_fmadd_ps(
                    _mm_fmadd_ps(_mm_set1_ps(1.0f / 120.0f), m,
                                 _mm_set1_ps(1.0f / 24.0f)),
                    x2,
                    _mm_fmadd_ps(_mm_fmadd_ps(_mm_set1_ps(1.0f / 5040.0f), m,
                                              _mm_set1_ps(1.0f / 720.0f)),
                                 x4,
                                 _mm_fmadd_ps(_mm_set1_ps(1.0f / 6.0f), m,
                                              _mm_set1_ps(1.0f / 2.0f)))),
                x2, m),
            e, e);
        // Handle special cases.
        z = _mm_blendv_ps(
            z, _mm_castsi128_ps(_mm_set1_epi32(0x7F800000)),
            _mm_cmp_ps(v, _mm_set1_ps(38.682357788f), _CMP_GE_OQ));
        return _mm_andnot_ps(
            _mm_cmp_ps(v, _mm_set1_ps(-38.381324768f), _CMP_LE_OQ), z);
    }
    /// Compute element-wise exponentiation, 10^x.
    ///
    /// See __m128 overload for more docs.
    static inline __m256 Pow10(__m256 v)
    {
        // Divide input by log10(2) and break into integer and fractional
        // components.
        const __m256 r =
            _mm256_round_ps(_mm256_mul_ps(v, _mm256_set1_ps(3.321928024f)), 8);
        __m256 m = _mm256_fmadd_ps(r, _mm256_set1_ps(-0.301025391f), v);
        m = _mm256_fmadd_ps(r, _mm256_set1_ps(-4.60503907e-6f), m);
        m = _mm256_mul_ps(m, _mm256_set1_ps(2.302585125f));
        // Compute Taylor approximation of 10^b.
        const __m256 x2 = _mm256_mul_ps(m, m);
        const __m256 x4 = _mm256_mul_ps(x2, x2);
        const __m256 e = _mm256_min_ps(_mm256_castsi256_ps(_mm256_slli_epi32(
                                           _mm256_castps_si256(_mm256_add_ps(
                                               r, _mm256_set1_ps(8388735.0f))),
                                           23)),
                                       _mm256_set1_ps(3.40282346639e38f));
        __m256 z = _mm256_fmadd_ps(
            _mm256_fmadd_ps(
                _mm256_fmadd_ps(
                    _mm256_fmadd_ps(_mm256_set1_ps(1.0f / 120.0f), m,
                                    _mm256_set1_ps(1.0f / 24.0f)),
                    x2,
                    _mm256_fmadd_ps(
                        _mm256_fmadd_ps(_mm256_set1_ps(1.0f / 5040.0f), m,
                                        _mm256_set1_ps(1.0f / 720.0f)),
                        x4,
                        _mm256_fmadd_ps(_mm256_set1_ps(1.0f / 6.0f), m,
                                        _mm256_set1_ps(1.0f / 2.0f)))),
                x2, m),
            e, e);
        // Handle special cases.
        z = _mm256_blendv_ps(
            z, _mm256_castsi256_ps(_mm256_set1_epi32(0x7F800000)),
            _mm256_cmp_ps(v, _mm256_set1_ps(38.682357788f), _CMP_GE_OQ));
        return _mm256_andnot_ps(
            _mm256_cmp_ps(v, _mm256_set1_ps(-38.381324768f), _CMP_LE_OQ), z);
    }
    /// Compute element-wise sine in radians, aka sin(x).
    ///
    /// The approach is a range-reduction followed by a fairly straightforward
    /// Taylor approximation, with the only complication being that we select
    /// from two different Taylor approximations (sin and cos) to avoid
    /// excessive error in the flat part.
    ///
    /// The outputs of the trig functions (Sin, Cos, Tan) have very little
    /// significance in the presence of very large inputs, because the period
    /// with radian inputs is irrational and not representable as a float (nor
    /// as a double).
    ///
    /// The proper way to do trig functions is with units of Turns (where 1 Turn
    /// == 360 deg), but we have the codebase we have and must be pragmatic.
    ///
    /// The solution to using these functions over a wide range of input, then,
    /// is to maintain a rational modulus and range-reduce accurately first, on
    /// the client side, and only convert to radians to call these functions.
    ///
    /// We provide the following limited-domain relative error guarantee,
    /// exercised in vecmath_test.cc:
    ///
    /// For |x| < 1e6, worst-case relative error < 1.3e-7.
    static inline __m128 Sin(__m128 v)
    {
        const __m128 n =
            _mm_fmadd_ps(v, _mm_set1_ps(0.63661977f), _mm_set1_ps(12582912.0f));
        const __m128 d = _mm_sub_ps(n, _mm_set1_ps(12582912.0f));
        const __m128 x = _mm_fmadd_ps(
            d, _mm_set1_ps(1.71512451e-15f),
            _mm_fmadd_ps(d, _mm_set1_ps(-7.549790126e-8f),
                         _mm_fmadd_ps(d, _mm_set1_ps(-1.570796251f), v)));
        const __m128 x2 = _mm_mul_ps(x, x);
        const __m128 s = _mm_fnmadd_ps(
            _mm_fmadd_ps(x2,
                         _mm_fmadd_ps(x2, _mm_set1_ps(1.951529589e-4f),
                                      _mm_set1_ps(-8.33216087e-3f)),
                         _mm_set1_ps(0.166666546f)),
            _mm_mul_ps(x, x2), x);
        const __m128 c = _mm_fmadd_ps(
            x2,
            _mm_fmadd_ps(
                x2,
                _mm_fmadd_ps(x2,
                             _mm_fmadd_ps(x2, _mm_set1_ps(2.4433157e-5f),
                                          _mm_set1_ps(-1.3887316e-3f)),
                             _mm_set1_ps(0.04166664568f)),
                _mm_set1_ps(-0.5f)),
            _mm_set1_ps(1.0f));
        const __m128 r = _mm_blendv_ps(
            s, c, _mm_castsi128_ps(_mm_slli_epi32(_mm_castps_si128(n), 31)));
        return _mm_xor_ps(r, _mm_castsi128_ps(_mm_slli_epi32(
                                 _mm_srli_epi32(_mm_castps_si128(n), 1), 31)));
    }
    /// Compute element-wise sine in radians, aka sin(x).
    ///
    /// For |x| < 1e6, worst-case relative error < 1.3e-7.
    ///
    /// See __m128 overload for more docs.
    static inline __m256 Sin(__m256 v)
    {
        const __m256 n = _mm256_fmadd_ps(v, _mm256_set1_ps(0.63661977f),
                                         _mm256_set1_ps(12582912.0f));
        const __m256 d = _mm256_sub_ps(n, _mm256_set1_ps(12582912.0f));
        const __m256 x = _mm256_fnmadd_ps(
            d, _mm256_set1_ps(-1.71512451e-15f),
            _mm256_fmadd_ps(
                d, _mm256_set1_ps(-7.549790126e-8f),
                _mm256_fmadd_ps(d, _mm256_set1_ps(-1.570796251f), v)));
        const __m256 x2 = _mm256_mul_ps(x, x);
        const __m256 s = _mm256_fnmadd_ps(
            _mm256_fmadd_ps(x2,
                            _mm256_fmadd_ps(x2, _mm256_set1_ps(1.951529589e-4f),
                                            _mm256_set1_ps(-8.33216087e-3f)),
                            _mm256_set1_ps(0.166666546f)),
            _mm256_mul_ps(x, x2), x);
        const __m256 c = _mm256_fmadd_ps(
            x2,
            _mm256_fmadd_ps(
                x2,
                _mm256_fmadd_ps(x2,
                                _mm256_fmadd_ps(x2,
                                                _mm256_set1_ps(2.4433157e-5f),
                                                _mm256_set1_ps(-1.3887316e-3f)),
                                _mm256_set1_ps(0.04166664568f)),
                _mm256_set1_ps(-0.5f)),
            _mm256_set1_ps(1.0f));
        const __m256 r = _mm256_blendv_ps(
            s, c,
            _mm256_castsi256_ps(_mm256_slli_epi32(_mm256_castps_si256(n), 31)));
        return _mm256_xor_ps(
            r, _mm256_castsi256_ps(_mm256_slli_epi32(
                   _mm256_srli_epi32(_mm256_castps_si256(n), 1), 31)));
    }
    /// Compute element-wise cosine in radians, aka cos(x).
    ///
    /// For |x| < 1e6, worst-case relative error < 1.3e-7.
    ///
    /// See Sin() for more docs.
    static inline __m128 Cos(__m128 v)
    {
        const __m128 n =
            _mm_fmadd_ps(v, _mm_set1_ps(0.63661977f), _mm_set1_ps(12582912.0f));
        const __m128 d = _mm_sub_ps(n, _mm_set1_ps(12582912.0f));
        const __m128 x = _mm_fmadd_ps(
            d, _mm_set1_ps(1.71512451e-15f),
            _mm_fmadd_ps(d, _mm_set1_ps(-7.549790126e-8f),
                         _mm_fmadd_ps(d, _mm_set1_ps(-1.570796251f), v)));
        const __m128 x2 = _mm_mul_ps(x, x);
        const __m128 s = _mm_fmsub_ps(
            _mm_fmadd_ps(x2,
                         _mm_fmadd_ps(x2, _mm_set1_ps(1.951529589e-4f),
                                      _mm_set1_ps(-8.33216087e-3f)),
                         _mm_set1_ps(0.166666546f)),
            _mm_mul_ps(x, x2), x);
        const __m128 c = _mm_fmadd_ps(
            x2,
            _mm_fmadd_ps(
                x2,
                _mm_fmadd_ps(x2,
                             _mm_fmadd_ps(x2, _mm_set1_ps(2.4433157e-5f),
                                          _mm_set1_ps(-1.3887316e-3f)),
                             _mm_set1_ps(0.04166664568f)),
                _mm_set1_ps(-0.5f)),
            _mm_set1_ps(1.0f));
        const __m128 r = _mm_blendv_ps(
            c, s, _mm_castsi128_ps(_mm_slli_epi32(_mm_castps_si128(n), 31)));
        return _mm_xor_ps(r, _mm_castsi128_ps(_mm_slli_epi32(
                                 _mm_srli_epi32(_mm_castps_si128(n), 1), 31)));
    }
    /// Compute element-wise cosine in radians, aka cos(x).
    ///
    /// For |x| < 1e6, worst-case relative error < 1.3e-7.
    ///
    /// See __m128 overload for more docs.
    static inline __m256 Cos(__m256 v)
    {
        const __m256 n = _mm256_fmadd_ps(v, _mm256_set1_ps(0.63661977f),
                                         _mm256_set1_ps(12582912.0f));
        const __m256 d = _mm256_sub_ps(n, _mm256_set1_ps(12582912.0f));
        const __m256 x = _mm256_fnmadd_ps(
            d, _mm256_set1_ps(-1.71512451e-15f),
            _mm256_fmadd_ps(
                d, _mm256_set1_ps(-7.549790126e-8f),
                _mm256_fmadd_ps(d, _mm256_set1_ps(-1.570796251f), v)));
        const __m256 x2 = _mm256_mul_ps(x, x);
        const __m256 s = _mm256_fmsub_ps(
            _mm256_fmadd_ps(x2,
                            _mm256_fmadd_ps(x2, _mm256_set1_ps(1.951529589e-4f),
                                            _mm256_set1_ps(-8.33216087e-3f)),
                            _mm256_set1_ps(0.166666546f)),
            _mm256_mul_ps(x, x2), x);
        const __m256 c = _mm256_fmadd_ps(
            x2,
            _mm256_fmadd_ps(
                x2,
                _mm256_fmadd_ps(x2,
                                _mm256_fmadd_ps(x2,
                                                _mm256_set1_ps(2.4433157e-5f),
                                                _mm256_set1_ps(-1.3887316e-3f)),
                                _mm256_set1_ps(0.04166664568f)),
                _mm256_set1_ps(-0.5f)),
            _mm256_set1_ps(1.0f));
        const __m256 r = _mm256_blendv_ps(
            c, s,
            _mm256_castsi256_ps(_mm256_slli_epi32(_mm256_castps_si256(n), 31)));
        return _mm256_xor_ps(
            r, _mm256_castsi256_ps(_mm256_slli_epi32(
                   _mm256_srli_epi32(_mm256_castps_si256(n), 1), 31)));
    }
    /// Compute element-wise sine and cosine in radians, aka sin(x) and cos(x),
    /// for about the same cost as computing either by itself.
    ///
    /// For |x| < 1e6, worst-case relative error < 1.3e-7.
    ///
    /// See Sin() and Cos() for more docs.
    static inline void Sincos(__m128 &__restrict s, __m128 &__restrict c,
                              const __m128 v)
    {
        const __m128 n =
            _mm_fmadd_ps(v, _mm_set1_ps(0.63661977f), _mm_set1_ps(12582912.0f));
        const __m128 d = _mm_sub_ps(n, _mm_set1_ps(12582912.0f));
        const __m128 x = _mm_fmadd_ps(
            d, _mm_set1_ps(1.71512451e-15f),
            _mm_fmadd_ps(d, _mm_set1_ps(-7.549790126e-8f),
                         _mm_fmadd_ps(d, _mm_set1_ps(-1.570796251f), v)));
        const __m128 x2 = _mm_mul_ps(x, x);
        const __m128 s1 = _mm_fnmadd_ps(
            _mm_fmadd_ps(x2,
                         _mm_fmadd_ps(x2, _mm_set1_ps(1.951529589e-4f),
                                      _mm_set1_ps(-8.33216087e-3f)),
                         _mm_set1_ps(0.166666546f)),
            _mm_mul_ps(x, x2), x);
        const __m128 c1 = _mm_fmadd_ps(
            x2,
            _mm_fmadd_ps(
                x2,
                _mm_fmadd_ps(x2,
                             _mm_fmadd_ps(x2, _mm_set1_ps(2.4433157e-5f),
                                          _mm_set1_ps(-1.3887316e-3f)),
                             _mm_set1_ps(0.04166664568f)),
                _mm_set1_ps(-0.5f)),
            _mm_set1_ps(1.0f));
        const __m128 m =
            _mm_castsi128_ps(_mm_slli_epi32(_mm_castps_si128(n), 31));
        const __m128 s2 = _mm_blendv_ps(s1, c1, m);
        const __m128 s3 =
            _mm_xor_ps(s2, _mm_castsi128_ps(_mm_slli_epi32(
                               _mm_srli_epi32(_mm_castps_si128(n), 1), 31)));
        const __m128 c2 = _mm_xor_ps(s3, _mm_xor_ps(m, _mm_xor_ps(c1, s1)));
        s = s3;
        c = c2;
    }
    /// Compute element-wise sine and cosine in radians, aka sin(x) and cos(x),
    /// for about the same cost as computing either by itself.
    ///
    /// For |x| < 1e6, worst-case relative error < 1.3e-7.
    ///
    /// See Sin() and Cos() for more docs.
    static inline void Sincos(__m256 &__restrict s, __m256 &__restrict c,
                              const __m256 v)
    {
        const __m256 n = _mm256_fmadd_ps(v, _mm256_set1_ps(0.63661977f),
                                         _mm256_set1_ps(12582912.0f));
        const __m256 d = _mm256_sub_ps(n, _mm256_set1_ps(12582912.0f));
        const __m256 x = _mm256_fmadd_ps(
            d, _mm256_set1_ps(1.71512451e-15f),
            _mm256_fmadd_ps(
                d, _mm256_set1_ps(-7.549790126e-8f),
                _mm256_fmadd_ps(d, _mm256_set1_ps(-1.570796251f), v)));
        const __m256 x2 = _mm256_mul_ps(x, x);
        const __m256 s1 = _mm256_fnmadd_ps(
            _mm256_fmadd_ps(x2,
                            _mm256_fmadd_ps(x2, _mm256_set1_ps(1.951529589e-4f),
                                            _mm256_set1_ps(-8.33216087e-3f)),
                            _mm256_set1_ps(0.166666546f)),
            _mm256_mul_ps(x, x2), x);
        const __m256 c1 = _mm256_fmadd_ps(
            x2,
            _mm256_fmadd_ps(
                x2,
                _mm256_fmadd_ps(x2,
                                _mm256_fmadd_ps(x2,
                                                _mm256_set1_ps(2.4433157e-5f),
                                                _mm256_set1_ps(-1.3887316e-3f)),
                                _mm256_set1_ps(0.04166664568f)),
                _mm256_set1_ps(-0.5f)),
            _mm256_set1_ps(1.0f));
        const __m256 m =
            _mm256_castsi256_ps(_mm256_slli_epi32(_mm256_castps_si256(n), 31));
        const __m256 s2 = _mm256_blendv_ps(s1, c1, m);
        const __m256 s3 = _mm256_xor_ps(
            s2, _mm256_castsi256_ps(_mm256_slli_epi32(
                    _mm256_srli_epi32(_mm256_castps_si256(n), 1), 31)));
        const __m256 c2 =
            _mm256_xor_ps(s3, _mm256_xor_ps(m, _mm256_xor_ps(c1, s1)));
        s = s3;
        c = c2;
    }
    /// Compute element-wise tangent in radians, aka tan(x).
    ///
    /// For |x| < 1e6, worst-case relative error < 2.5e-7.
    ///
    /// See Sin() for more docs.
    static inline __m128 Tan(__m128 v)
    {
        __m128 s;
        __m128 c;
        Sincos(s, c, v);
        return _mm_div_ps(s, c);
    }
    /// Compute element-wise tangent in radians, aka tan(x).
    ///
    /// For |x| < 1e6, worst-case relative error < 2.5e-7.
    ///
    /// See __m128 overload for more docs.
    static inline __m256 Tan(__m256 v)
    {
        __m256 s;
        __m256 c;
        Sincos(s, c, v);
        return _mm256_div_ps(s, c);
    }
    /// Compute element-wise inverse cosine in radians, aka arccos(x), acos(x),
    /// or cos^-1(x).
    ///
    /// Output range [0, pi], matching the standard library implementation.
    ///
    /// In vecmath_test.cc we check all 2^32 float inputs against the standard
    /// library implementation, requiring worst-case relative error < 2.3e-7.
    ///
    /// The approach is a fairly straightforward Taylor approximation, with the
    /// only complication being that we select from two different Taylor
    /// approximations depending on whether the absolute value of the input is
    /// above or below 0.5f, to keep the error from growing too large as we get
    /// far away from the center of the Taylor expansion.
    static inline __m128 Acos(__m128 v)
    {
        const __m128 a =
            _mm_and_ps(v, _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF)));
        const __m128 t = _mm_fnmadd_ps(_mm_set1_ps(0.5f), a, _mm_set1_ps(0.5f));
        const __m128 m = _mm_cmp_ps(a, _mm_set1_ps(0.5f), _CMP_GT_OQ);
        const __m128 x = _mm_blendv_ps(_mm_mul_ps(v, v), t, m);
        const __m128 y = _mm_blendv_ps(a, _mm_sqrt_ps(t), m);
        const __m128 x2 = _mm_mul_ps(x, x);
        const __m128 r = _mm_fmadd_ps(
            _mm_fmadd_ps(
                _mm_fmadd_ps(_mm_set1_ps(2.4181311049e-2f), x,
                             _mm_set1_ps(4.5470025998e-2f)),
                x2,
                _mm_fmadd_ps(_mm_set1_ps(4.2163199048e-2f), _mm_mul_ps(x2, x2),
                             _mm_fmadd_ps(_mm_set1_ps(7.4953002686e-2f), x,
                                          _mm_set1_ps(0.166667524f)))),
            _mm_mul_ps(x, y), y);
        const __m128 s =
            _mm_and_ps(v, _mm_castsi128_ps(_mm_set1_epi32(0x80000000)));
        const __m128 p =
            _mm_xor_ps(s, _mm_add_ps(_mm_set1_ps(-1.0f),
                                     _mm_and_ps(m, _mm_set1_ps(3.0f))));
        const __m128 b = _mm_sub_ps(
            _mm_and_ps(m, _mm_xor_ps(s, _mm_set1_ps(1.57079632679f))),
            _mm_set1_ps(1.57079632679f));
        return _mm_fmsub_ps(p, r, b);
    }
    /// Compute element-wise inverse cosine in radians, aka arccos(x), acos(x),
    /// or cos^-1(x).
    ///
    /// Output range [0, pi], matching the standard library implementation.
    ///
    /// See __m128 overload for more docs.
    static inline __m256 Acos(__m256 v)
    {
        const __m256 a = _mm256_and_ps(
            v, _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF)));
        const __m256 t =
            _mm256_fnmadd_ps(_mm256_set1_ps(0.5f), a, _mm256_set1_ps(0.5f));
        const __m256 m = _mm256_cmp_ps(a, _mm256_set1_ps(0.5f), _CMP_GT_OQ);
        const __m256 x = _mm256_blendv_ps(_mm256_mul_ps(v, v), t, m);
        const __m256 y = _mm256_blendv_ps(a, _mm256_sqrt_ps(t), m);
        const __m256 x2 = _mm256_mul_ps(x, x);
        const __m256 r = _mm256_fmadd_ps(
            _mm256_fmadd_ps(
                _mm256_fmadd_ps(_mm256_set1_ps(2.4181311049e-2f), x,
                                _mm256_set1_ps(4.5470025998e-2f)),
                x2,
                _mm256_fmadd_ps(
                    _mm256_set1_ps(4.2163199048e-2f), _mm256_mul_ps(x2, x2),
                    _mm256_fmadd_ps(_mm256_set1_ps(7.4953002686e-2f), x,
                                    _mm256_set1_ps(0.166667524f)))),
            _mm256_mul_ps(x, y), y);
        const __m256 s = _mm256_and_ps(
            v, _mm256_castsi256_ps(_mm256_set1_epi32(0x80000000)));
        const __m256 p = _mm256_xor_ps(
            s, _mm256_add_ps(_mm256_set1_ps(-1.0f),
                             _mm256_and_ps(m, _mm256_set1_ps(3.0f))));
        const __m256 b = _mm256_sub_ps(
            _mm256_and_ps(m, _mm256_xor_ps(s, _mm256_set1_ps(1.57079632679f))),
            _mm256_set1_ps(1.57079632679f));
        return _mm256_fmsub_ps(p, r, b);
    }
    /// Compute element-wise inverse sine in radians, aka arcsin(x), asin(x), or
    /// sin^-1(x).
    ///
    /// Output range [-pi/2, pi/2], matching the standard library
    /// implementation.
    ///
    /// See Acos() for more docs, as the approach is the same.
    static inline __m128 Asin(__m128 v)
    {
        const __m128 a =
            _mm_and_ps(v, _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF)));
        const __m128 t = _mm_fnmadd_ps(_mm_set1_ps(0.5f), a, _mm_set1_ps(0.5f));
        const __m128 m = _mm_cmp_ps(a, _mm_set1_ps(0.5f), _CMP_GT_OQ);
        const __m128 x = _mm_blendv_ps(_mm_mul_ps(v, v), t, m);
        const __m128 y = _mm_blendv_ps(a, _mm_sqrt_ps(t), m);
        const __m128 x2 = _mm_mul_ps(x, x);
        const __m128 r = _mm_fmadd_ps(
            _mm_fmadd_ps(
                _mm_fmadd_ps(_mm_set1_ps(2.4181311049e-2f), x,
                             _mm_set1_ps(4.5470025998e-2f)),
                x2,
                _mm_fmadd_ps(_mm_set1_ps(4.2163199048e-2f), _mm_mul_ps(x2, x2),
                             _mm_fmadd_ps(_mm_set1_ps(7.4953002686e-2f), x,
                                          _mm_set1_ps(0.166667524f)))),
            _mm_mul_ps(x, y), y);
        return _mm_xor_ps(
            _mm_blendv_ps(r,
                          _mm_fmadd_ps(_mm_set1_ps(-2.0f), r,
                                       _mm_set1_ps(1.57079632679f)),
                          m),
            _mm_and_ps(v, _mm_castsi128_ps(_mm_set1_epi32(0x80000000))));
    }
    /// Compute element-wise inverse sine in radians, aka arcsin(x), asin(x), or
    /// sin^-1(x).
    ///
    /// Output range [-pi/2, pi/2], matching the standard library
    /// implementation.
    ///
    /// See __m128 overload for more docs.
    static inline __m256 Asin(__m256 v)
    {
        const __m256 a = _mm256_and_ps(
            v, _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF)));
        const __m256 t =
            _mm256_fnmadd_ps(_mm256_set1_ps(0.5f), a, _mm256_set1_ps(0.5f));
        const __m256 m = _mm256_cmp_ps(a, _mm256_set1_ps(0.5f), _CMP_GT_OQ);
        const __m256 x = _mm256_blendv_ps(_mm256_mul_ps(v, v), t, m);
        const __m256 y = _mm256_blendv_ps(a, _mm256_sqrt_ps(t), m);
        const __m256 x2 = _mm256_mul_ps(x, x);
        const __m256 r = _mm256_fmadd_ps(
            _mm256_fmadd_ps(
                _mm256_fmadd_ps(_mm256_set1_ps(2.4181311049e-2f), x,
                                _mm256_set1_ps(4.5470025998e-2f)),
                x2,
                _mm256_fmadd_ps(
                    _mm256_set1_ps(4.2163199048e-2f), _mm256_mul_ps(x2, x2),
                    _mm256_fmadd_ps(_mm256_set1_ps(7.4953002686e-2f), x,
                                    _mm256_set1_ps(0.166667524f)))),
            _mm256_mul_ps(x, y), y);
        return _mm256_xor_ps(
            _mm256_blendv_ps(r,
                             _mm256_fmadd_ps(_mm256_set1_ps(-2.0f), r,
                                             _mm256_set1_ps(1.57079632679f)),
                             m),
            _mm256_and_ps(v,
                          _mm256_castsi256_ps(_mm256_set1_epi32(0x80000000))));
    }
    /// Compute element-wise inverse arctangent of y/x in radians, aka atan2(y,
    /// x).
    ///
    /// Output range [-pi, pi], matching the standard library implementation.
    ///
    /// In vecmath_atan2_test.cc we check the full unit circle and a wide
    /// variety of other float inputs against the standard library
    /// implementation, requiring worst-case relative error < 2.5e-7.
    ///
    /// Special cases are IEEE-754 compliant, except that:
    /// - Denormals may be treated as zero.
    /// - Finite but extremely large inputs, above 1e38, may exhibit larger than
    /// the advertised 2.5e-7
    ///   worst-case relative error. See comment inside implementation. This
    ///   note does not apply to infinities, which are handled correctly.
    /// - The signedness of zero as a return value may be arbitrary.
    ///
    /// The approach is a fairly straightforward Taylor approximation, with the
    /// only complication being that we select from two different Taylor
    /// approximations depending on whether the absolute value of the input is
    /// above or below Sqrt(2) - 1, to keep the error from growing too large as
    /// we get far away from the center of the Taylor expansion.
    static inline __m128 Atan2(__m128 y, __m128 x)
    {
        // Octant reduction.
        const __m128 absY =
            _mm_and_ps(y, _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF)));
        const __m128 absX =
            _mm_and_ps(x, _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF)));
        const __m128 didSwap = _mm_cmp_ps(absY, absX, _CMP_GT_OQ);
        const __m128 u = _mm_blendv_ps(absX, absY, didSwap);
        const __m128 v = _mm_xor_ps(u, _mm_xor_ps(absX, absY));
        // Determine which Taylor approx to use and compute initial ratio as v/u
        // or (v-u)/(v+u) accordingly.
        const __m128 c = _mm_cmp_ps(_mm_mul_ps(u, _mm_set1_ps(0.414213562f)), v,
                                    _CMP_NGT_UQ);
        const __m128 f = _mm_div_ps(_mm_sub_ps(v, _mm_and_ps(c, u)),
                                    _mm_add_ps(u, _mm_and_ps(c, v)));
        // To handle all inputs, even those above 1e38, to within the
        // advertised 2.5e-7 worst-case relative error, replace the above
        // computation of 'f' with the following: const __m128 d = _mm_div_ps(v,
        // u); const __m128 f = _mm_div_ps(
        //     _mm_sub_ps(d, _mm_and_ps(c, _mm_set1_ps(1.0f))),
        //     _mm_add_ps(_mm_and_ps(d, c), _mm_set1_ps(1.0f)));
        // Handle special case Abs(x) == Abs(y) == inf.
        const __m128 t = _mm_andnot_ps(
            _mm_castsi128_ps(_mm_cmpeq_epi32(
                _mm_max_epu32(_mm_castps_si128(u), _mm_castps_si128(v)),
                _mm_set1_epi32(0x7F800000))),
            f);
        // Compute Taylor approximation.
        const __m128 t2 = _mm_mul_ps(t, t);
        __m128 r = _mm_add_ps(
            _mm_fmadd_ps(
                _mm_fmadd_ps(_mm_fmadd_ps(_mm_set1_ps(0.080537445f), t2,
                                          _mm_set1_ps(-0.13877685f)),
                             _mm_mul_ps(t2, t2),
                             _mm_fmadd_ps(_mm_set1_ps(0.19977711f), t2,
                                          _mm_set1_ps(-0.33332949f))),
                _mm_mul_ps(t2, t), t),
            _mm_and_ps(c, _mm_set1_ps(0.78539816f)));
        // Octant fixup and more special cases.
        r = _mm_andnot_ps(
            _mm_cmp_ps(_mm_or_ps(x, y), _mm_setzero_ps(), _CMP_EQ_OQ), r);
        r = _mm_blendv_ps(r, _mm_sub_ps(_mm_set1_ps(1.57079633f), r), didSwap);
        r = _mm_blendv_ps(r, _mm_sub_ps(_mm_set1_ps(3.1415927f), r), x);
        return _mm_xor_ps(
            r, _mm_and_ps(y, _mm_castsi128_ps(_mm_set1_epi32(0x80000000))));
    }
    /// Compute element-wise inverse arctangent of y/x in radians, aka atan2(y,
    /// x).
    ///
    /// Output range [-pi, pi], matching the standard library implementation.
    ///
    /// Special cases are IEEE-754 compliant, except that:
    /// - Denormals may be treated as zero.
    /// - Finite but extremely large inputs, above 1e38, may exhibit larger than
    /// the advertised 2.5e-7
    ///   worst-case relative error. See comment inside implementation. This
    ///   note does not apply to infinities, which are handled correctly.
    /// - The signedness of zero as a return value may be arbitrary.
    ///
    /// See __m128 overload for more docs.
    static inline __m256 Atan2(__m256 y, __m256 x)
    {
        // Octant reduction.
        const __m256 absY = _mm256_and_ps(
            y, _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF)));
        const __m256 absX = _mm256_and_ps(
            x, _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF)));
        const __m256 didSwap = _mm256_cmp_ps(absY, absX, _CMP_GT_OQ);
        const __m256 u = _mm256_blendv_ps(absX, absY, didSwap);
        const __m256 v = _mm256_xor_ps(u, _mm256_xor_ps(absX, absY));
        // Determine which Taylor approx to use and compute initial ratio as v/u
        // or (v-u)/(v+u) accordingly.
        const __m256 c = _mm256_cmp_ps(
            _mm256_mul_ps(u, _mm256_set1_ps(0.414213562f)), v, _CMP_NGT_UQ);
        const __m256 f = _mm256_div_ps(_mm256_sub_ps(v, _mm256_and_ps(c, u)),
                                       _mm256_add_ps(u, _mm256_and_ps(c, v)));
        // To handle all inputs, even those above 1e38, to within the
        // advertised 2.5e-7 worst-case relative error, replace the above
        // computation of 'f' with the following: const __m256 d =
        // _mm256_div_ps(v, u); const __m256 f = _mm256_div_ps(
        //     _mm256_sub_ps(d, _mm256_and_ps(c, _mm256_set1_ps(1.0f))),
        //     _mm256_add_ps(_mm256_and_ps(d, c), _mm256_set1_ps(1.0f)));
        // Handle special case Abs(x) == Abs(y) == inf.
        const __m256 t =
            _mm256_andnot_ps(_mm256_castsi256_ps(_mm256_cmpeq_epi32(
                                 _mm256_max_epu32(_mm256_castps_si256(u),
                                                  _mm256_castps_si256(v)),
                                 _mm256_set1_epi32(0x7F800000))),
                             f);
        // Compute Taylor approximation.
        const __m256 t2 = _mm256_mul_ps(t, t);
        __m256 r = _mm256_add_ps(
            _mm256_fmadd_ps(
                _mm256_fmadd_ps(_mm256_fmadd_ps(_mm256_set1_ps(0.080537445f),
                                                t2,
                                                _mm256_set1_ps(-0.13877685f)),
                                _mm256_mul_ps(t2, t2),
                                _mm256_fmadd_ps(_mm256_set1_ps(0.19977711f), t2,
                                                _mm256_set1_ps(-0.33332949f))),
                _mm256_mul_ps(t2, t), t),
            _mm256_and_ps(c, _mm256_set1_ps(0.78539816f)));
        // Octant fixup and more special cases.
        r = _mm256_andnot_ps(
            _mm256_cmp_ps(_mm256_or_ps(x, y), _mm256_setzero_ps(), _CMP_EQ_OQ),
            r);
        r = _mm256_blendv_ps(r, _mm256_sub_ps(_mm256_set1_ps(1.57079633f), r),
                             didSwap);
        r = _mm256_blendv_ps(r, _mm256_sub_ps(_mm256_set1_ps(3.1415927f), r),
                             x);
        return _mm256_xor_ps(
            r, _mm256_and_ps(
                   y, _mm256_castsi256_ps(_mm256_set1_epi32(0x80000000))));
    }
} // namespace Drone::Perf
#endif // __AVX2__
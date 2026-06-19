/// @file src/utils/perf_util/perf_util_core.h
#pragma once
#include <cstdint>
#include <cstdio>
namespace Drone::Perf
{
#undef U8
#undef U16
#undef U32
#undef U64
#undef U128
#undef I8
#undef I16
#undef I32
#undef I64
#undef I128
    //! @cond Doxygen_Suppress
    using U8 = uint8_t;
    using U16 = uint16_t;
    using U32 = uint32_t;
    using U64 = uint64_t;
#ifdef __SIZEOF_INT128__
    // NOTE: if the need arises to divide U128s, use
    // https://github.com/komrad36/FastDivide128.
    using U128 = __uint128_t;
#endif
    using I8 = int8_t;
    using I16 = int16_t;
    using I32 = int32_t;
    using I64 = int64_t;
#ifdef __SIZEOF_INT128__
    using I128 = __int128_t;
#endif
    //! @endcond
#undef LIKELY
/// Expect-taken branch hint.
#define LIKELY(x) __builtin_expect(static_cast<bool>(x), 1)
#undef UNLIKELY
/// Expect-not-taken branch hint.
#define UNLIKELY(x) __builtin_expect(static_cast<bool>(x), 0)
#undef UNREACHABLE
/// Unreachable hint.
#define UNREACHABLE() __builtin_unreachable()
#undef ASSUME
/// Assume hint.
#define ASSUME(x) __builtin_assume(static_cast<bool>(x))
#undef ALWAYS_INLINE
/// Force inline.
#define ALWAYS_INLINE __attribute__((always_inline))
#undef NEVER_INLINE
/// Force not inline.
#define NEVER_INLINE __attribute__((noinline))
#undef FALLTHROUGH
/// Tell compilers and humans that a missing break statement in a switch-case is
/// deliberate.
#define FALLTHROUGH [[fallthrough]]
#undef FORCE_ALIGN_STACK
/// Force 32-byte stack alignment.
#define FORCE_ALIGN_STACK __attribute__((force_align_arg_pointer))
#undef MAYBE_UNUSED
/// Maybe unused.
#define MAYBE_UNUSED [[maybe_unused]]
#undef WARN_UNUSED
/// Add to a struct or class definition (e.g., struct WARN_UNUSED Foo {...}) to
/// warn if an instance of that struct or class goes unused. This does not occur
/// by default even though we run -Wunused.
#define WARN_UNUSED __attribute__((warn_unused))
#undef ARRAY_COUNT
/// Element count. Fear not, clang warns if erroneously used on a pointer.
#define ARRAY_COUNT(a)                                                         \
    (static_cast<void>(sizeof(a) / sizeof(*(a))), sizeof(a) / sizeof((a)[0]))
#undef ZERO_ARRAY
/// Memset all of array 'a' with zero bytes.
#define ZERO_ARRAY(a) __builtin_memset(&(a), 0, ARRAY_COUNT(a) * sizeof((a)[0]))
#undef FILL
/// Set 'n' elements at 'p' to 'x'. Like memset, but supports any element size,
/// not just bytes.
#define FILL(p, x, n)                                                          \
    do                                                                         \
    {                                                                          \
        U64 _n = (n);                                                          \
        __typeof(*(p)) _x = (x);                                               \
        __typeof(*(p)) *_p = (p);                                              \
        for (U64 _i = 0; _i < _n; ++_i)                                        \
            _p[_i] = _x;                                                       \
    } while (0)
#undef FILL_ARRAY
/// Set every element of 'a' to 'x'.
#define FILL_ARRAY(a, x) FILL(a, x, ARRAY_COUNT(a))
#undef STRLEN
/// Strlen of a null-terminated string literal.
#define STRLEN(a)                                                              \
    (static_cast<void>(static_cast<const char *>(a)), ARRAY_COUNT(a) - 1)
#undef SUBSTR
/// Returns a bool indicating whether null-terminated string literal 's' occurs
/// at 'p'.
#define SUBSTR(p, s) (!__builtin_strncmp((p), (s), STRLEN(s)))
#undef DEBUG_BREAK
#if defined(__x86_64__) || defined(__i386__)
/// Debug break.
#define DEBUG_BREAK() __asm__ volatile("int3")
#elif defined(__aarch64__)
/// Debug break.
#define DEBUG_BREAK() __asm__ volatile("brk 0")
#else
/// Debug break.
#define DEBUG_BREAK() __asm__ volatile("bkpt")
#endif
#ifdef __FILE_NAME__
#define SX_PERF_FILENAME __FILE_NAME__
#else
    /// Extract filename from file path.
    static constexpr inline const char *SxPerfGetFileName(const char *s)
    {
        const char *o = s;
        while (const char c = *s++)
            o = c == '/' || c == '\\' ? s : o;
        return o;
    }
/// Extract filename from file path.
#define SX_PERF_FILENAME SxPerfGetFileName(__FILE__)
#endif
#if defined(ENABLE_SX_ASSERTS)
/// Enforce that asserts are off in production. You may freely remove this check
/// as desired for local work, but do not check it in.
#ifndef SX_TEST_TARGET
#error SX_ASSERT enabled in production
#endif
/// Assert.
#define SX_ASSERT(a)                                                           \
    do                                                                         \
        if (UNLIKELY(!(a)))                                                    \
        {                                                                      \
            fprintf(stderr, "%s:%d|ASSERT FAILED: %s\n", SX_PERF_FILENAME,     \
                    __LINE__, #a);                                             \
            fflush(stdout);                                                    \
            fflush(stderr);                                                    \
            DEBUG_BREAK();                                                     \
        }                                                                      \
    while (0)
/// Assert and print.
#define SX_ASSERTF(a, s, ...)                                                  \
    do                                                                         \
        if (UNLIKELY(!(a)))                                                    \
        {                                                                      \
            fprintf(stderr, "%s:%d|ASSERT FAILED: %s. " s "\n",                \
                    SX_PERF_FILENAME, __LINE__, #a, ##__VA_ARGS__);            \
            fflush(stdout);                                                    \
            fflush(stderr);                                                    \
            DEBUG_BREAK();                                                     \
        }                                                                      \
    while (0)
/// Halt.
#define SX_HALT()                                                              \
    do                                                                         \
    {                                                                          \
        fprintf(stderr, "%s:%d|HALT\n", SX_PERF_FILENAME, __LINE__);           \
        fflush(stdout);                                                        \
        fflush(stderr);                                                        \
        DEBUG_BREAK();                                                         \
    } while (0)
/// Halt and print.
#define SX_HALTF(s, ...)                                                       \
    do                                                                         \
    {                                                                          \
        fprintf(stderr, "%s:%d|HALT: " s "\n", SX_PERF_FILENAME, __LINE__,     \
                ##__VA_ARGS__);                                                \
        fflush(stdout);                                                        \
        fflush(stderr);                                                        \
        DEBUG_BREAK();                                                         \
    } while (0)
#else
/// Assert (disabled).
#define SX_ASSERT(a)                                                           \
    do                                                                         \
    {                                                                          \
    } while (0)
/// Assert and print (disabled).
#define SX_ASSERTF(a, s, ...)                                                  \
    do                                                                         \
    {                                                                          \
    } while (0)
/// Halt (disabled).
#define SX_HALT()                                                              \
    do                                                                         \
    {                                                                          \
    } while (0)
/// Halt and print (disabled).
#define SX_HALTF(s, ...)                                                       \
    do                                                                         \
    {                                                                          \
    } while (0)
#endif
} // namespace Drone::Perf
/// @file src/utils/perf_util/perf_util_constants.h
#pragma once
namespace Drone::Perf
{
    /// Spherical earth radius using value from H3 cell library, in meters.
    static constexpr float kEarthSphereRadiusH3 = 6371007.2f;
    /// WGS84 earth semimajor axis (radius in the XY plane at the equator), in
    /// meters.
    static constexpr float kEarthSemiMajorAxis = 6378137.0f;
    /// WGS84 earth semiminor axis (radius in the Z direction), in meters.
    static constexpr float kEarthSemiMinorAxis = 6356752.314245f;
    /// Radius of geostationary orbit, in meters.
    static constexpr float kRadiusGeo = 42164169.0f;
} // namespace Drone::Perf
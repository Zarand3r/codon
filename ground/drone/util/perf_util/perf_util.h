/// @file src/utils/perf_util/perf_util.h
#pragma once
#include "src/utils/perf_util/perf_util_aarch64.h"
#include "src/utils/perf_util/perf_util_avx2.h"
#include "src/utils/perf_util/perf_util_constants.h"
#include "src/utils/perf_util/perf_util_core.h"
#include "src/utils/perf_util/perf_util_math.h"
#include "src/utils/perf_util/perf_util_profile.h"
/// @file src/utils/perf_util/perf_util_profile.h
#pragma once
#include "src/utils/perf_util/perf_util_core.h"
#include <chrono>
#include <cstdio>
namespace Drone::Perf
{
#ifdef ENABLE_SX_PROFILE
    class ProfileSimpleNugget
    {
    public:
        ProfileSimpleNugget(const char *pFile, const char *pFunc,
                            const int line)
            : m_pFile(pFile), m_pFunc(pFunc),
              m_start(std::chrono::high_resolution_clock::now()), m_line(line)
        {}
        ~ProfileSimpleNugget()
        {
            const std::chrono::high_resolution_clock::time_point end =
                std::chrono::high_resolution_clock::now();
            const U64 ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               end - m_start)
                               .count();
            printf("SXPROF|%s@%s:%d|%luns\n", m_pFile, m_pFunc, m_line, ns);
            fflush(stdout);
        }

    private:
        const char *m_pFile;
        const char *m_pFunc;
        const std::chrono::high_resolution_clock::time_point m_start;
        const int m_line;
    };
    class ProfileAccumStatic
    {
    private:
        template <U64 kProfileAccumPrintFreq>
        friend class ProfileAccumNugget;

    public:
        ProfileAccumStatic(const char *pFile, const char *pFunc, const int line)
            : m_pFile(pFile), m_pFunc(pFunc), m_totalNs(0), m_numVisits(0),
              m_line(line)
        {}

    private:
        const char *m_pFile;
        const char *m_pFunc;
        U64 m_totalNs;
        U64 m_numVisits;
        const int m_line;
    };
    template <U64 kProfileAccumPrintFreq = 1>
    class ProfileAccumNugget
    {
        static_assert(kProfileAccumPrintFreq);

    public:
        ProfileAccumNugget(ProfileAccumStatic &pas)
            : m_pas(pas), m_start(std::chrono::high_resolution_clock::now())
        {}
        ~ProfileAccumNugget()
        {
            const std::chrono::high_resolution_clock::time_point end =
                std::chrono::high_resolution_clock::now();
            const U64 ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               end - m_start)
                               .count();
            ++m_pas.m_numVisits;
            m_pas.m_totalNs += ns;
            if (m_pas.m_numVisits % kProfileAccumPrintFreq == 0)
            {
                printf("SXPROF|%s@%s:%d|%lux|cur %luns|tot %luns|avg %luns\n",
                       m_pas.m_pFile, m_pas.m_pFunc, m_pas.m_line,
                       m_pas.m_numVisits, ns, m_pas.m_totalNs,
                       m_pas.m_totalNs / m_pas.m_numVisits);
                fflush(stdout);
            }
        }

    private:
        ProfileAccumStatic &m_pas;
        const std::chrono::high_resolution_clock::time_point m_start;
    };
#define SX_CONCAT1(x, y) x##_##y
#define SX_CONCAT2(x, y) SX_CONCAT1(x, y)
/// Simple profile. Add to any scope block to profile from that point to the end
/// of the scope block. Performs no accumulation or tracking of multiple
/// invocations across time.
#define PROFILE_SIMPLE()                                                       \
    const ProfileSimpleNugget SX_CONCAT2(profileNugget, __LINE__)(             \
        SX_PERF_FILENAME, __FUNCTION__, __LINE__)
/// Accumulated profile. Add to any scope block to profile from that point to
/// the end of the scope block. Tracks multiple invocations across time and
/// accumulates statistics accordingly. Pass no arguments to report on every
/// invocation, or an integer n to only report on every nth invocation.
#define PROFILE_ACCUM(...)                                                     \
    static ProfileAccumStatic SX_CONCAT2(profileAccumStatic, __LINE__)(        \
        SX_PERF_FILENAME, __FUNCTION__, __LINE__);                             \
    const ProfileAccumNugget<__VA_ARGS__> SX_CONCAT2(profileAccumNugget,       \
                                                     __LINE__)(                \
        SX_CONCAT2(profileAccumStatic, __LINE__))
#else
/// Simple profile (disabled).
#define PROFILE_SIMPLE()                                                       \
    do                                                                         \
    {                                                                          \
    } while (0)
/// Accumulated profile (disabled).
#define PROFILE_ACCUM(...)                                                     \
    do                                                                         \
    {                                                                          \
    } while (0)
#endif // #ifdef ENABLE_SX_PROFILE
} // namespace Drone::Perf
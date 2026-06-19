#pragma once

// Sac: the flight-software assertion / early-return vocabulary used pervasively as
// the control-flow backbone (validate every input, fail closed). Minimal core set
// for L0/L1; the typed numeric variants and dbprintf family are added when a
// consumer needs them (R1 — fill only what is used).
//
// Contract from usage:
//   SacAbortIfNot(cond, ret)  — if !cond: log + `return ret;`
//   SacAbortIf(cond, ret)     — if  cond: log + `return ret;`
//   SacAssert(cond)           — hard assert: log + abort on false (also in release)
//   SacDebugAssert(cond)      — assert only in debug builds (compiled out under NDEBUG)

#include <cstdio>
#include <cstdlib>

namespace Drone
{
    // Structured failure report to stderr (rate-unlimited; callers that fire this
    // in a tight loop should guard themselves — see failure_policy).
    inline void sac_report(const char *file, int line, const char *expr)
    {
        std::fprintf(stderr, "%s:%d|SAC FAILED: %s\n", file, line, expr);
        std::fflush(stderr);
    }
} // namespace Drone

#define SacAbortIfNot(cond, ret)                                               \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(!(cond), 0))                                      \
        {                                                                      \
            ::Drone::sac_report(__FILE__, __LINE__, #cond);                    \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define SacAbortIf(cond, ret)                                                  \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect((cond), 0))                                       \
        {                                                                      \
            ::Drone::sac_report(__FILE__, __LINE__, #cond);                    \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define SacAssert(cond)                                                        \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(!(cond), 0))                                      \
        {                                                                      \
            ::Drone::sac_report(__FILE__, __LINE__, #cond);                    \
            std::abort();                                                      \
        }                                                                      \
    } while (0)

#ifdef NDEBUG
#define SacDebugAssert(cond)                                                   \
    do                                                                         \
    {                                                                          \
    } while (0)
#else
#define SacDebugAssert(cond) SacAssert(cond)
#endif

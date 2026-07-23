#pragma once

// Fsw: the flight-software assertion / early-return vocabulary used pervasively as
// the control-flow backbone (validate every input, fail closed). Minimal core set
// for L0/L1; the typed numeric variants and dbprintf family are added when a
// consumer needs them (R1 — fill only what is used).
//
// Contract from usage:
//   FswAbortIfNot(cond, ret)  — if !cond: log + `return ret;`
//   FswAbortIf(cond, ret)     — if  cond: log + `return ret;`
//   FswAssert(cond)           — hard assert: log + abort on false (also in release)
//   FswDebugAssert(cond)      — assert only in debug builds (compiled out under NDEBUG)

#include <cstdio>
#include <cstdlib>

namespace Drone
{
    // Structured failure report to stderr (rate-unlimited; callers that fire this
    // in a tight loop should guard themselves — see failure_policy).
    inline void fsw_report(const char *file, int line, const char *expr)
    {
        std::fprintf(stderr, "%s:%d|FSW FAILED: %s\n", file, line, expr);
        std::fflush(stderr);
    }
} // namespace Drone

#define FswAbortIfNot(cond, ret)                                               \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(!(cond), 0))                                      \
        {                                                                      \
            ::Drone::fsw_report(__FILE__, __LINE__, #cond);                    \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define FswAbortIf(cond, ret)                                                  \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(!!(cond), 0))                                     \
        {                                                                      \
            ::Drone::fsw_report(__FILE__, __LINE__, #cond);                    \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define FswAssert(cond)                                                        \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(!(cond), 0))                                      \
        {                                                                      \
            ::Drone::fsw_report(__FILE__, __LINE__, #cond);                    \
            std::abort();                                                      \
        }                                                                      \
    } while (0)

#ifdef NDEBUG
#define FswDebugAssert(cond)                                                   \
    do                                                                         \
    {                                                                          \
    } while (0)
#else
#define FswDebugAssert(cond) FswAssert(cond)
#endif

// The logging / context / typed-comparison layer (dbnprintf, FswPrefix, FswStackFrame,
// FswAbortIfEq*/Neq*/OpUint64, FswMsgAbortIf*, FswIf/FswIfNot). NOTE: this include is
// unconditional — consumers of fsw.h DO get <string>/<cstdarg> transitively; the file
// split is organizational (assert core readable on its own), not a dependency cut.
#include "src/bullwinkle/all/core/fsw_log.h"

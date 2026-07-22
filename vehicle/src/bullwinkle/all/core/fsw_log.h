#pragma once

// Fsw logging / context layer — the diagnostic half of the fsw vocabulary that the
// Slate consumers use pervasively but that the ultra-core `fsw.h` (asserts only)
// deliberately omits. `fsw.h` includes this at its end, so anything with `fsw.h` gets
// the full vocabulary; the split just keeps <string>/<cstdarg> and the printf family
// out of the assert core.
//
// Cold path only: every symbol here is on a failure/log/build path (guarded by an
// abort/`FswIf` check), never in a RUNTIME hot method.
//
// Contract from usage:
//   dbnprintf(n, fmt, ...)          — bounded (<= n bytes) stderr printf, no prefix
//   dbstring(s)                     — write a literal string to stderr
//   FswPrefix()                     — print a "file:line" prefix (no newline) for the
//                                     line a following dbnprintf(": ...\n") completes
//   FswIf(cond) / FswIfNot(cond)    — the condition, hinted unlikely; the caller logs
//                                     + returns in the taken branch
//   FswAbortIfEqInt / EqUint64      — if (a == b) [typed]: log + return ret
//   FswAbortIfNeq / NeqInt / NeqInt64 / NeqDouble — if (a != b): log + return ret
//   FswAbortIfOpUint64(a, op, b, r) — if (a op b) [u64]: log + return ret
//   FswMsgAbortIf / FswMsgAbortIfNot(cond, ret, n, fmt, ...) — conditional log+return
//   FswStackFrame::get_current_stack_frame() -> nullptr unless a frame is pushed

#include "src/bullwinkle/all/core/drone_types.h"

#include <cstdarg>
#include <cstdio>
#include <string>

namespace Drone
{
    // Bounded formatted write to stderr. `n` caps the emitted length; the buffer is
    // fixed, so very large `n` is clamped (cold-path diagnostic — never truncates a
    // correctness signal, only a log line).
    inline void dbnprintf(int n, const char *fmt, ...)
        __attribute__((format(printf, 2, 3)));
    inline void dbnprintf(int n, const char *fmt, ...)
    {
        char buf[1024];
        size_t cap = sizeof(buf);
        if (n > 0 && static_cast<size_t>(n) < cap)
        {
            cap = static_cast<size_t>(n) + 1; // room for NUL within the cap
        }
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(buf, cap, fmt, ap);
        va_end(ap);
        std::fputs(buf, stderr);
        std::fflush(stderr);
    }

    // Verbosity-gated bounded printf: `level` is a verbosity threshold (emitted only
    // when >= the runtime verbosity, which defaults to 0 = emit). `n` bounds length.
    inline void dbvnprintf(int level, int n, const char *fmt, ...)
        __attribute__((format(printf, 3, 4)));
    inline void dbvnprintf(int level, int n, const char *fmt, ...)
    {
        static int verbosity = 0; // raise to see higher-level diagnostics
        if (level > verbosity)
        {
            return;
        }
        char buf[1024];
        size_t cap = sizeof(buf);
        if (n > 0 && static_cast<size_t>(n) < cap)
        {
            cap = static_cast<size_t>(n) + 1;
        }
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(buf, cap, fmt, ap);
        va_end(ap);
        std::fputs(buf, stderr);
        std::fflush(stderr);
    }

    // Write a literal string (no formatting).
    inline void dbstring(const char *s)
    {
        std::fputs(s, stderr);
        std::fflush(stderr);
    }

    // The "file:line" prefix a following dbnprintf(": ...") completes into a full line.
    inline void fsw_prefix(const char *file, int line)
    {
        std::fprintf(stderr, "%s:%d", file, line);
    }

    // A failed binary check: "file:line|FSW FAILED: <lhs> <op> <rhs>".
    inline void fsw_report_cmp(const char *file, int line, const char *lhs,
                               const char *op, const char *rhs)
    {
        std::fprintf(stderr, "%s:%d|FSW FAILED: %s %s %s\n", file, line, lhs, op, rhs);
        std::fflush(stderr);
    }

    // Verbose-diagnostics gate (off by default). FswOnVerbose(...) runs its argument
    // only when this is enabled, so extra per-check logging costs nothing in the
    // normal path but can be switched on for debugging.
    inline bool &fsw_verbose_flag()
    {
        static bool enabled = false;
        return enabled;
    }
    inline bool fsw_verbose() { return fsw_verbose_flag(); }

    // A comparison/relation abort report: "file:line|<abort_type>: <str_1> <str_2>".
    // Aborts if is_fatal. Used by the typed fsw_if_* helpers (e.g. Handle comparison).
    inline void report_abort(const char *abort_type, const char *str_1,
                             const char *str_2, const char *file, int line,
                             bool is_fatal)
    {
        std::fprintf(stderr, "%s:%d|%s: %s %s\n", file, line, abort_type, str_1,
                     str_2);
        std::fflush(stderr);
        if (is_fatal)
        {
            std::abort();
        }
    }

    // Log one operand value alongside a report_abort (verbose diagnostics).
    inline void fsw_arg(UINT64 value)
    {
        std::fprintf(stderr, "  arg: %llu\n", value);
    }

    // A conditional-abort message: "file:line|FSW FAILED: <formatted>".
    inline void fsw_msg(const char *file, int line, int n, const char *fmt, ...)
        __attribute__((format(printf, 4, 5)));
    inline void fsw_msg(const char *file, int line, int n, const char *fmt, ...)
    {
        char buf[1024];
        size_t cap = sizeof(buf);
        if (n > 0 && static_cast<size_t>(n) < cap)
        {
            cap = static_cast<size_t>(n) + 1;
        }
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(buf, cap, fmt, ap);
        va_end(ap);
        std::fprintf(stderr, "%s:%d|FSW FAILED: %s\n", file, line, buf);
        std::fflush(stderr);
    }

    // Per-thread diagnostic context. Nothing pushes a frame yet, so
    // get_current_stack_frame() returns nullptr — callers must handle that (e.g. the
    // token accountant records no location when it is null). The API is the real
    // shape so a future enhancement can push frames from the abort macros.
    class FswStackFrame
    {
    public:
        std::string file_location{};

        static const FswStackFrame *get_current_stack_frame() { return current(); }

    private:
        static const FswStackFrame *&current()
        {
            static thread_local const FswStackFrame *c = nullptr;
            return c;
        }
    };
} // namespace Drone

// --- control-flow / logging macros -----------------------------------------------

// Condition passthrough, hinted unlikely (fail-closed style). The caller logs and
// returns in the taken branch: `if (FswIf(bad)) { FswPrefix(); dbnprintf(...); ... }`.
#define FswIf(cond) (__builtin_expect(!!(cond), 0))
#define FswIfNot(cond) (__builtin_expect(!(cond), 0))
// The (unlikely-hinted) truth of a != b — the caller logs + acts in the taken branch.
#define FswIfNeq(a, b) (__builtin_expect(!((a) == (b)), 0))

// Print the call-site prefix; a following dbnprintf(": ...\n") completes the line.
#define FswPrefix() ::Drone::fsw_prefix(__FILE__, __LINE__)

// Run `expr` only when verbose diagnostics are enabled (compiled in, runtime-gated).
#define FswOnVerbose(expr)                                                     \
    do                                                                         \
    {                                                                          \
        if (::Drone::fsw_verbose())                                            \
        {                                                                      \
            expr;                                                              \
        }                                                                      \
    } while (0)

// Log one operand value (used inside FswOnVerbose next to a report_abort).
#define FswArg(x) ::Drone::fsw_arg(x)

// Typed comparison aborts. Operands are cast to the suffix type so the comparison is
// unambiguous (no -Wsign-compare) and the intent is explicit.
#define FswAbortIfEqInt(a, b, ret)                                             \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(static_cast<long>(a) == static_cast<long>(b), 0)) \
        {                                                                      \
            ::Drone::fsw_report_cmp(__FILE__, __LINE__, #a, "==", #b);         \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define FswAbortIfEqUint64(a, b, ret)                                          \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(static_cast<UINT64>(a) == static_cast<UINT64>(b), \
                             0))                                               \
        {                                                                      \
            ::Drone::fsw_report_cmp(__FILE__, __LINE__, #a, "==", #b);         \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define FswAbortIfNeq(a, b, ret)                                               \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(!((a) == (b)), 0))                                \
        {                                                                      \
            ::Drone::fsw_report_cmp(__FILE__, __LINE__, #a, "!=", #b);         \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define FswAbortIfNeqInt(a, b, ret)                                            \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(static_cast<long>(a) != static_cast<long>(b), 0)) \
        {                                                                      \
            ::Drone::fsw_report_cmp(__FILE__, __LINE__, #a, "!=", #b);         \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define FswAbortIfNeqInt64(a, b, ret)                                          \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(static_cast<INT64>(a) != static_cast<INT64>(b),   \
                             0))                                               \
        {                                                                      \
            ::Drone::fsw_report_cmp(__FILE__, __LINE__, #a, "!=", #b);         \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define FswAbortIfNeqUint64(a, b, ret)                                         \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(static_cast<UINT64>(a) != static_cast<UINT64>(b), \
                             0))                                               \
        {                                                                      \
            ::Drone::fsw_report_cmp(__FILE__, __LINE__, #a, "!=", #b);         \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define FswAbortIfNeqDouble(a, b, ret)                                         \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(static_cast<double>(a) != static_cast<double>(b), \
                             0))                                               \
        {                                                                      \
            ::Drone::fsw_report_cmp(__FILE__, __LINE__, #a, "!=", #b);         \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

// If (a op b) as UINT64: op is passed as a macro argument, e.g. (shard_size, >, max).
#define FswAbortIfOpUint64(a, op, b, ret)                                      \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(static_cast<UINT64>(a) op static_cast<UINT64>(b), \
                             0))                                               \
        {                                                                      \
            ::Drone::fsw_report_cmp(__FILE__, __LINE__, #a, #op, #b);          \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

// Conditional abort with a bounded formatted message (n = max bytes).
#define FswMsgAbortIf(cond, ret, n, ...)                                       \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(!!(cond), 0))                                     \
        {                                                                      \
            ::Drone::fsw_msg(__FILE__, __LINE__, (n), __VA_ARGS__);            \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

#define FswMsgAbortIfNot(cond, ret, n, ...)                                    \
    do                                                                         \
    {                                                                          \
        if (__builtin_expect(!(cond), 0))                                      \
        {                                                                      \
            ::Drone::fsw_msg(__FILE__, __LINE__, (n), __VA_ARGS__);            \
            return (ret);                                                      \
        }                                                                      \
    } while (0)

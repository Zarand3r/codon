// Contract test for the fsw logging/context layer (core/fsw_log.h, included via
// core/fsw.h). The macros are control-flow: the assertions here are that the taken
// branch returns the given value and the not-taken branch falls through. Diagnostic
// output goes to stderr (visually confirmed; not asserted).
#include "src/bullwinkle/all/core/fsw.h"

#include <cassert>
#include <cstdio>
#include <string>

using namespace Drone;

// Each helper returns a sentinel iff its abort macro fired, else 0 — so a test can
// assert the control-flow contract without capturing stderr.
static int eq_int(int a, int b)
{
    FswAbortIfEqInt(a, b, 1);
    return 0;
}
static int eq_u64(UINT64 a, UINT64 b)
{
    FswAbortIfEqUint64(a, b, 1);
    return 0;
}
static int neq(size_t a, size_t b)
{
    FswAbortIfNeq(a, b, 1);
    return 0;
}
static int neq_int(int a, int b)
{
    FswAbortIfNeqInt(a, b, 1);
    return 0;
}
static int neq_i64(INT64 a, INT64 b)
{
    FswAbortIfNeqInt64(a, b, 1);
    return 0;
}
static int neq_double(double a, double b)
{
    FswAbortIfNeqDouble(a, b, 1);
    return 0;
}
static int op_u64_gt(UINT64 a, UINT64 b)
{
    FswAbortIfOpUint64(a, >, b, 1);
    return 0;
}
static int op_u64_ge(UINT64 a, UINT64 b)
{
    FswAbortIfOpUint64(a, >=, b, 1);
    return 0;
}
static int msg_if(bool cond)
{
    FswMsgAbortIf(cond, 1, 100, "msg if: %d", 42);
    return 0;
}
static int msg_if_not(bool cond)
{
    FswMsgAbortIfNot(cond, 1, 100, "msg if not: %s", "x");
    return 0;
}

int main()
{
    std::fprintf(stderr, "--- fsw_log_test: expected diagnostic lines below ---\n");

    // Equality aborts.
    assert(eq_int(5, 5) == 1);
    assert(eq_int(5, 6) == 0);
    assert(eq_u64(0u, 0u) == 1);
    assert(eq_u64(1u, 0u) == 0);

    // Inequality aborts (generic + typed).
    assert(neq(3u, 4u) == 1);
    assert(neq(4u, 4u) == 0);
    assert(neq_int(-1, 2) == 1);
    assert(neq_int(2, 2) == 0);
    assert(neq_i64(-5, -6) == 1);
    assert(neq_i64(-6, -6) == 0);
    assert(neq_double(1.0, 2.0) == 1);
    assert(neq_double(2.0, 2.0) == 0);

    // Relational abort with the operator as a macro argument.
    assert(op_u64_gt(10u, 5u) == 1); // 10 > 5 -> abort
    assert(op_u64_gt(5u, 10u) == 0);
    assert(op_u64_ge(5u, 5u) == 1); // 5 >= 5 -> abort
    assert(op_u64_ge(4u, 5u) == 0);

    // Message aborts.
    assert(msg_if(true) == 1);
    assert(msg_if(false) == 0);
    assert(msg_if_not(false) == 1); // aborts when cond is false
    assert(msg_if_not(true) == 0);

    // FswIf / FswIfNot are the (unlikely-hinted) truth of the condition.
    assert(FswIf(1 == 1));
    assert(!FswIf(1 == 2));
    assert(FswIfNot(1 == 2));
    assert(!FswIfNot(1 == 1));
    assert(FswIfNeq(3, 4));
    assert(!FswIfNeq(4, 4));

    // Verbose-gated diagnostics: off by default (FswOnVerbose runs nothing), on when
    // enabled. report_abort (non-fatal) and FswArg are the verbose helpers.
    {
        int ran = 0;
        FswOnVerbose(ran = 1);
        assert(ran == 0); // verbose off -> body skipped
        Drone::fsw_verbose_flag() = true;
        FswOnVerbose(ran = 1);
        assert(ran == 1); // verbose on -> body runs
        FswOnVerbose(Drone::report_abort("TEST", "a", "b", __FILE__, __LINE__, false));
        FswOnVerbose(FswArg((UINT64)123));
        Drone::fsw_verbose_flag() = false;
    }

    // dbnprintf bounds output; dbstring writes a literal; FswPrefix + FswStackFrame
    // are exercised for compile + runtime safety.
    dbnprintf(8, "truncated to eight bytes but the format is much longer %d\n", 7);
    dbstring("dbstring literal\n");
    FswPrefix();
    dbstring(": prefixed line\n");
    assert(FswStackFrame::get_current_stack_frame() == nullptr); // none pushed yet

    std::printf("fsw_log_test: OK\n");
    return 0;
}

// TDD (P0): fsw contract — early-return + assert macros.
// NOTE: the FswAbortIfNot(false)/FswAbortIf(true) paths intentionally log to
// stderr; that is expected. PASS/FAIL is reported on stdout.
#include "src/bullwinkle/all/core/fsw.h"

#include <cassert>
#include <cstdio>

namespace
{
    bool returns_false_when_cond_false(bool ok)
    {
        FswAbortIfNot(ok, false); // !ok -> return false
        return true;
    }
    int returns_neg1_when_bad(bool bad)
    {
        FswAbortIf(bad, -1); // bad -> return -1
        return 0;
    }

    // Category: the condition is a non-bool scalar (pointer / int). The macros must
    // coerce it to bool — a raw pointer must not be handed to __builtin_expect(long).
    bool require_ptr(const int *p)
    {
        FswAbortIfNot(p, false); // !p
        return true;
    }
    int bail_if_ptr(const int *p)
    {
        FswAbortIf(p, -1); // !!p
        return 0;
    }
    int bail_if_count(int n)
    {
        FswAbortIf(n, -1); // !!n
        return 0;
    }
}

int main()
{
    assert(returns_false_when_cond_false(true) == true);  // proceeds
    assert(returns_false_when_cond_false(false) == false); // early-returns ret
    assert(returns_neg1_when_bad(false) == 0);             // proceeds
    assert(returns_neg1_when_bad(true) == -1);             // early-returns ret

    // Pointer / int conditions (the bug class): must compile and behave correctly.
    int x = 0;
    assert(require_ptr(&x) == true);    // non-null -> proceeds
    assert(require_ptr(nullptr) == false);
    assert(bail_if_ptr(nullptr) == 0);  // null -> proceeds
    assert(bail_if_ptr(&x) == -1);      // non-null -> early-return
    assert(bail_if_count(0) == 0);
    assert(bail_if_count(5) == -1);

    FswAssert(1 + 1 == 2);      // true: no abort
    FswDebugAssert(2 + 2 == 4); // true: no abort

    std::printf("fsw_test: PASS\n");
    return 0;
}

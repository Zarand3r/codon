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
}

int main()
{
    assert(returns_false_when_cond_false(true) == true);  // proceeds
    assert(returns_false_when_cond_false(false) == false); // early-returns ret
    assert(returns_neg1_when_bad(false) == 0);             // proceeds
    assert(returns_neg1_when_bad(true) == -1);             // early-returns ret

    FswAssert(1 + 1 == 2);      // true: no abort
    FswDebugAssert(2 + 2 == 4); // true: no abort

    std::printf("fsw_test: PASS\n");
    return 0;
}

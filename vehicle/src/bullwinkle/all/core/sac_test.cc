// TDD (P0): sac contract — early-return + assert macros.
// NOTE: the SacAbortIfNot(false)/SacAbortIf(true) paths intentionally log to
// stderr; that is expected. PASS/FAIL is reported on stdout.
#include "src/bullwinkle/all/core/sac.h"

#include <cassert>
#include <cstdio>

namespace
{
    bool returns_false_when_cond_false(bool ok)
    {
        SacAbortIfNot(ok, false); // !ok -> return false
        return true;
    }
    int returns_neg1_when_bad(bool bad)
    {
        SacAbortIf(bad, -1); // bad -> return -1
        return 0;
    }
}

int main()
{
    assert(returns_false_when_cond_false(true) == true);  // proceeds
    assert(returns_false_when_cond_false(false) == false); // early-returns ret
    assert(returns_neg1_when_bad(false) == 0);             // proceeds
    assert(returns_neg1_when_bad(true) == -1);             // early-returns ret

    SacAssert(1 + 1 == 2);      // true: no abort
    SacDebugAssert(2 + 2 == 4); // true: no abort

    std::printf("sac_test: PASS\n");
    return 0;
}

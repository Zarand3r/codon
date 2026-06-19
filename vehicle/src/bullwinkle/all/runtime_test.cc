// TDD (P0): runtime.h annotation macros are no-ops that compile in their use sites.
#include "src/bullwinkle/all/runtime.h"

#include <cstdio>

namespace
{
    // RUNTIME as a trailing method annotation.
    struct Component
    {
        int tick() const RUNTIME { return 1; }
        // INFRASTRUCTURE(T) wraps an init-time member's type (identity).
        INFRASTRUCTURE(int) counter = 0;
    };
}

// HOTSYNC_EXEMPT placed before a definition.
HOTSYNC_EXEMPT int helper() { return 2; }

int main()
{
    Component c;
    if (c.tick() + helper() + c.counter == 3)
    {
        std::printf("runtime_test: PASS\n");
        return 0;
    }
    std::printf("runtime_test: FAIL\n");
    return 1;
}

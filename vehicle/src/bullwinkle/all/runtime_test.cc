// Contract test for runtime.h: the annotation macros are no-ops that compile in
// their use positions. The only non-trivial requirement is that INFRASTRUCTURE(...)
// is variadic, so it wraps a type containing commas (e.g. a templated member).
#include "src/bullwinkle/all/runtime.h"

#include <cstdio>
#include <map>

namespace
{
    struct Component
    {
        // RUNTIME as a trailing method annotation.
        int tick() const RUNTIME { return 1; }

        // INFRASTRUCTURE(T) wraps an init-time member's type. The comma in the
        // template argument only compiles if the macro is variadic (__VA_ARGS__).
        INFRASTRUCTURE(std::map<int, int>) table;
        INFRASTRUCTURE(int) counter = 0;
    };
}

// HOTSYNC_EXEMPT placed before a definition.
HOTSYNC_EXEMPT int helper() { return 2; }

int main()
{
    Component c;
    c.table[7] = 9; // the comma-typed member is a usable std::map
    if (c.tick() + helper() + c.counter + c.table[7] == 12)
    {
        std::printf("runtime_test: PASS\n");
        return 0;
    }
    std::printf("runtime_test: FAIL\n");
    return 1;
}

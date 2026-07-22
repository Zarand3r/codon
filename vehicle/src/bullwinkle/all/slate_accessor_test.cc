// Behavioral test for the validator/accessor subsystem (slate_accessor.h).
// Contract inferred from SlateBuilder.h (binds a validator by down-casting a
// Handle<SlateValidator> to Handle<SlateTypedValidator<T>> and calling validate)
// and slate_accessor.h's own docs (SlateAccessor: read-converts, store() runs the
// validator, value unchanged on rejection).
#include "src/bullwinkle/all/slate_accessor.h"

#include <cassert>
#include <cstdio>

using namespace Drone;

int main()
{
    // Default validator is a no-op / not usable.
    {
        slate_validator_t v;
        assert(v.is_noop());
        assert(!v.is_usable());
    }

    // function_validator builds a usable, type-erased validator that a typed
    // Handle can recover (the SlateBuilder bind path).
    {
        // Accept only even ints; on accept, write the value through `val`.
        slate_validator_t v = function_validator<int>(
            [](const int &nv, int &val) {
                if (nv % 2 != 0)
                {
                    return false; // reject odd, leave val unchanged
                }
                val = nv;
                return true;
            });
        assert(!v.is_noop() && v.is_usable());

        // Down-cast recovers the typed validator (mirrors SlateBuilder).
        Handle<SlateTypedValidator<int>> typed;
        assert(typed.assign_casted(v.internal));
        int slot = 7;
        assert(typed->validate(4, slot) && slot == 4);   // accepted
        assert(!typed->validate(5, slot) && slot == 4);  // rejected, unchanged

        // Down-cast to the WRONG type fails cleanly (no crash).
        Handle<SlateTypedValidator<double>> wrong;
        assert(!wrong.assign_casted(v.internal));
    }

    // SlateAccessor over real storage: read converts, store validates.
    {
        int storage = 100;
        slate_validator_t v = function_validator<int>(
            [](const int &nv, int &val) {
                if (nv < 0)
                {
                    return false; // reject negatives
                }
                val = nv;
                return true;
            });

        SlateAccessor<int> acc(&storage, v);

        // Read path: implicit conversion + get().
        int read = acc;
        assert(read == 100);
        assert(acc.get() == 100);

        // Accepted write.
        assert(acc.store(42));
        assert(storage == 42);

        // Rejected write leaves the value unchanged, returns false.
        assert(!acc.store(-1));
        assert(storage == 42);

        // operator= is fire-and-forget: accepted change lands...
        acc = 55;
        assert(storage == 55);
        // ...rejected change is silently dropped (value unchanged).
        acc = -9;
        assert(storage == 55);
    }

    // Accessor with a no-op validator: store always fails (no validator to run),
    // matching "a validated element requires a validator to write".
    {
        int storage = 1;
        SlateAccessor<int> acc(&storage, slate_validator_t{});
        assert(!acc.store(2));
        assert(storage == 1);
    }

    std::printf("slate_accessor_test: OK\n");
    return 0;
}

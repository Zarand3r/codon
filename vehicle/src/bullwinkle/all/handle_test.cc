// Behavioral test for Handle<T> — the shared-ownership smart pointer used pervasively
// (Slate holds Handle<SlateMemory>, SlateBuilder holds Handle<SlateBuilderStoreInterface>,
// etc.). std::shared_ptr-backed: refcounted copies, explicit bool, deref/arrow,
// assume_ownership, is_unique, and a total order for use as map keys.
// (vehicle/src/bullwinkle/all/Handle.h)
#include "src/bullwinkle/all/Handle.h"

#include <cassert>
#include <cstdio>

using Drone::Handle;

namespace
{
    struct Widget
    {
        int v;
        explicit Widget(int v_) : v(v_) {}
    };
} // namespace

int main()
{
    // Default: empty.
    {
        Handle<Widget> h;
        assert(!h);
        assert(h.get() == nullptr);
    }

    // Owning: bool, deref, arrow, get, unique.
    {
        Handle<Widget> h(new Widget(42));
        assert(static_cast<bool>(h));
        assert(h.get() != nullptr);
        assert(h->v == 42);
        assert((*h).v == 42);
        assert(h.is_unique());
    }

    // Copy shares ownership: refcount rises, both non-unique, compare equal.
    {
        Handle<Widget> a(new Widget(7));
        Handle<Widget> b = a;
        assert(a == b);
        assert(!(a != b));
        assert(!a.is_unique() && !b.is_unique());
        b->v = 9;
        assert(a->v == 9); // same object
    }

    // Distinct objects compare unequal.
    {
        Handle<Widget> a(new Widget(1));
        Handle<Widget> b(new Widget(1));
        assert(a != b); // identity, not value
    }

    // assume_ownership replaces the managed object.
    {
        Handle<Widget> h;
        h.assume_ownership(new Widget(5));
        assert(h && h->v == 5);
        h.assume_ownership(new Widget(6));
        assert(h->v == 6);
    }

    std::printf("handle_test: OK\n");
    return 0;
}

// Consumer-compile gate (IMPLEMENTATION_PLAN §4: "the strongest integration test is
// compiling the real consumer"). slate_tokens.h is the first *imported* consumer of
// the slate_info / slate_type / slate_id contracts. Instantiating its token templates
// forces those contracts to be checked as the real code uses them — a mis-inferred
// type/id/permission surface fails to compile here instead of passing a mirror test.
//
// Compiled object-only (no link): slate_tokens.cc defines the accountant extern
// globals and pulls in the fsw logging layer (dbnprintf/FswPrefix/FswStackFrame/
// FswAbortIfNeq), which do not exist yet — so a full compile+link of slate_tokens.cc
// is a later increment. This witness validates everything reachable at compile time.
#include "src/bullwinkle/all/slate_tokens.h"

using namespace Drone;

namespace
{
    struct Payload
    {
        int a;
        double b;
    };
} // namespace

// odr-uses each token template so their bodies (ctor/copy/assign/get_type_id ->
// slate_type_id<T>) instantiate and type-check against slate_info.
void slate_tokens_compile_witness()
{
    ReadToken<int> r;
    WriteToken<double> w;
    WriteValidatorToken<Payload> v;

    // Exercise the members Slate/SlateBuilder call on tokens.
    const slate_type_t rt = r.type_id();
    const slate_type_t wt = w.type_id();
    const slate_type_t vt = v.type_id();
    (void)rt;
    (void)wt;
    (void)vt;

    // Access-flag statics resolve to compile-time bools.
    static_assert(ReadToken<int>::can_read, "read token reads");
    static_assert(!ReadToken<int>::can_write, "read token cannot write");
    static_assert(WriteToken<double>::can_write, "write token writes");
    static_assert(WriteValidatorToken<Payload>::can_validate, "validator validates");

    // Copy/assign paths.
    ReadToken<int> r2(r);
    r = r2;
    (void)r2;
}

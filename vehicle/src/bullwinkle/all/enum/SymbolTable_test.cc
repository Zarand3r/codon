// Contract test for SymbolTable — the bijective name<->value reflection table used
// for enum symbol lookup (`slate_shard_t_sym`, `slate_elem_access_t_sym`,
// StateMachine's `state_sym`/`ctask_sym`).
//
// Inferred contract from usage sites:
//   - default-constructible, copyable, comparable (SlateCombiner compares two tables)
//   - add(name, value) -> bool: inserts a name<->value pair; rejects a collision on
//     either the name or the value (it is a bijection), returns false without mutating
//   - raw_get(name, value&) -> bool: name -> value (StateMachine::lookup_state)
//   - raw_get(value, name&) -> bool: value -> name (StateMachine::lookup_state(uint))
//   - get(value) -> std::string: value -> name, or "" if absent (used directly in
//     printf error messages, so it must not assert on a miss)
//   - dump() const: debug print (smoke only)
//   - operator==/!=: equal iff the same set of name<->value mappings
// (vehicle/src/bullwinkle/all/enum/SymbolTable.h)
#include "src/bullwinkle/all/enum/SymbolTable.h"

#include <cassert>
#include <cstdio>
#include <string>

using Drone::SymbolTable;

int main()
{
    // Empty table: no lookups succeed; get() on a miss yields "".
    {
        SymbolTable t;
        uint v = 123;
        std::string n = "sentinel";
        assert(!t.raw_get("nope", v));
        assert(v == 123); // out param untouched on miss
        assert(!t.raw_get(7u, n));
        assert(n == "sentinel"); // out param untouched on miss
        assert(t.get(7u).empty());
    }

    // add() then round-trip both directions.
    {
        SymbolTable t;
        assert(t.add("static", 0u));
        assert(t.add("sync", 1u));
        assert(t.add("nonsync", 2u));

        uint v = 0;
        assert(t.raw_get("sync", v) && v == 1u);
        assert(t.raw_get("nonsync", v) && v == 2u);

        std::string n;
        assert(t.raw_get(0u, n) && n == "static");
        assert(t.raw_get(2u, n) && n == "nonsync");

        assert(t.get(1u) == "sync");
        assert(t.get(0u) == "static");
    }

    // Bijection: duplicate name OR duplicate value is rejected, table unchanged.
    {
        SymbolTable t;
        assert(t.add("a", 0u));
        assert(!t.add("a", 5u));  // duplicate name
        assert(!t.add("b", 0u));  // duplicate value
        // Neither rejected insert took effect.
        uint v = 99;
        assert(!t.raw_get("b", v));
        assert(v == 99);
        assert(t.get(0u) == "a");
        std::string n;
        assert(t.raw_get(0u, n) && n == "a");
    }

    // Equality: same mappings compare equal regardless of insertion order;
    // any difference compares unequal.
    {
        SymbolTable a;
        a.add("x", 1u);
        a.add("y", 2u);

        SymbolTable b;
        b.add("y", 2u);
        b.add("x", 1u);
        assert(a == b);
        assert(!(a != b));

        SymbolTable c;
        c.add("x", 1u);
        assert(a != c); // subset, not equal

        SymbolTable d;
        d.add("x", 1u);
        d.add("y", 3u); // same names, different value
        assert(a != d);

        // Copy is equal to its source.
        SymbolTable e = a;
        assert(e == a);
    }

    // dump() is a smoke test — must not crash on empty or populated.
    {
        SymbolTable t;
        t.dump();
        t.add("z", 42u);
        t.dump();
    }

    std::printf("SymbolTable_test: OK\n");
    return 0;
}

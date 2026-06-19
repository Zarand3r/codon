// TDD (P0): contract test for static_vector, written BEFORE the implementation.
// Contract inferred from present usage (R1 — fill only what is used):
//   - default-constructs empty
//   - push_back(const T&) appends; size() tracks the count
//   - operator[] reads and writes elements in order
//   - storage is inline (fixed capacity N, no heap)
// Sources: `static_vector<multi_command_t, 50> multi_command_v` (SlateCommandInterface.h);
//          `static_vector<source_t, ...>` (SlateCombiner).
#include "src/bullwinkle/all/static_vector.h"

#include <cassert>
#include <cstdio>

int main()
{
    Drone::static_vector<int, 4> v;
    assert(v.size() == 0); // empty on construction

    v.push_back(10);
    v.push_back(20);
    v.push_back(30);
    assert(v.size() == 3);                          // size tracks push_back
    assert(v[0] == 10 && v[1] == 20 && v[2] == 30); // operator[] reads in order

    v[1] = 25; // operator[] is a mutable reference
    assert(v[1] == 25);

    const Drone::static_vector<int, 4> &cv = v;
    assert(cv[0] == 10); // const operator[]

    // Inline, fixed-capacity storage — no heap allocation.
    static_assert(sizeof(Drone::static_vector<int, 4>) >= sizeof(int) * 4,
                  "static_vector must store its N elements inline");

    std::printf("static_vector_test: PASS\n");
    return 0;
}

// TDD (P0): sxtime contract — nano_t, sentinels, monotonic clock, sleep.
#include "src/bullwinkle/all/core/sxtime.h"

#include <cassert>
#include <cstdio>
#include <type_traits>

using Drone::billion;
using Drone::get_rel_time;
using Drone::nano_t;
using Drone::nano_t_max;
using Drone::nano_t_min;
using Drone::sxsleep;

static_assert(sizeof(nano_t) == 8, "nano_t is 64-bit");
static_assert(std::is_signed<nano_t>::value, "nano_t is signed");

int main()
{
    assert(nano_t_min < 0 && nano_t_max > 0 && nano_t_min < nano_t_max);
    assert(billion == 1000000000LL);

    const nano_t t0 = get_rel_time();
    const nano_t t1 = get_rel_time();
    assert(t1 >= t0); // monotonic, non-decreasing

    sxsleep(0);                 // non-positive returns immediately
    sxsleep(-5);                // negative returns immediately
    const nano_t a = get_rel_time();
    sxsleep(2 * 1000 * 1000);   // 2 ms
    const nano_t b = get_rel_time();
    assert(b - a >= 1 * 1000 * 1000); // at least ~1 ms elapsed

    std::printf("sxtime_test: PASS\n");
    return 0;
}

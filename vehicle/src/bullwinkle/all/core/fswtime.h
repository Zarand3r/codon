#pragma once

// Time vocabulary for the fixed-cadence control loop.
// Contract from usage: `nano_t` (signed 64-bit ns), `nano_t_min`/`nano_t_max`
// (the immediate-callback / never sentinels), `billion` (ns per second),
// `get_rel_time()` (monotonic now), `fswsleep(nano_t)` (sleep, used to phase-align
// non-sharing nodes).

#include "src/bullwinkle/all/core/drone_types.h"

#include <cstdint>
#include <ctime>

namespace Drone
{
    using nano_t = INT64;

    static constexpr nano_t nano_t_min = INT64_MIN; // "call me back immediately"
    static constexpr nano_t nano_t_max = INT64_MAX; // "never"
    static constexpr nano_t billion = 1000000000LL; // ns per second

    // Monotonic relative time in nanoseconds (not affected by wall-clock steps).
    inline nano_t get_rel_time()
    {
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<nano_t>(ts.tv_sec) * billion +
               static_cast<nano_t>(ts.tv_nsec);
    }

    // Sleep for the given duration. Non-positive durations return immediately.
    inline void fswsleep(nano_t ns)
    {
        if (ns <= 0)
        {
            return;
        }
        timespec ts{static_cast<time_t>(ns / billion),
                    static_cast<long>(ns % billion)};
        nanosleep(&ts, nullptr);
    }
} // namespace Drone

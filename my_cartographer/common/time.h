//
// Created by whitby on 2025-02-03.
//

#ifndef MY_CARTOGRAPHER_COMMON_TIME_H_
#define MY_CARTOGRAPHER_COMMON_TIME_H_

#include "my_cartographer/common/port.hpp"

#include <chrono>
#include <ostream>
#include <ratio>

namespace my_cartographer
{
  namespace common
  {

    constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
        (719162ll * 24ll * 60ll * 60ll);

    struct UniversalTimeScaleClock
    {
      using rep = int64;
      using period = std::ratio<1, 10000000>;
      using duration = std::chrono::duration<rep, period>;
      using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
      static constexpr bool is_steady = true;
    };

    // Represents Universal Time Scale durations and timestamps which are 64-bit
    // integers representing the 100 nanosecond ticks since the Epoch which is
    // January 1, 1 at the start of day in UTC.
    using Duration = UniversalTimeScaleClock::duration;
    using Time = UniversalTimeScaleClock::time_point;

    // Convenience functions to create common::Durations.
    Duration FromSeconds(double seconds);
    Duration FromMilliseconds(int64 milliseconds);

    // Returns the given duration in seconds.
    double ToSeconds(Duration duration);
    double ToSeconds(std::chrono::steady_clock::duration duration);

    // Creates a time from a Universal Time Scale.
    Time FromUniversal(int64 ticks);

    // Outputs the Universal Time Scale timestamp for a given Time.
    int64 ToUniversal(Time time);

    // For logging and unit tests, outputs the timestamp integer.
    std::ostream &operator<<(std::ostream &os, Time time);

    // CPU time consumed by the thread so far, in seconds.
    double GetThreadCpuTimeSeconds();

  } // namespace common
} // namespace my_cartographer

#endif // CARTOGRAPHER_COMMON_TIME_H_

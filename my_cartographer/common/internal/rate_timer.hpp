//
// Created by whitby on 2025-02-04.
//

#ifndef MY_CARTOGRAPHER_COMMON_RATE_TIMER_HPP_
#define MY_CARTOGRAPHER_COMMON_RATE_TIMER_HPP_

#include "my_cartographer/common/math.hpp"
#include "my_cartographer/common/port.hpp"
#include "my_cartographer/common/time.h"

#include <chrono>
#include <deque>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace my_cartographer
{
  namespace common
  {

    // Computes the rate at which pulses come in.
    template <typename ClockType = std::chrono::steady_clock>
    class RateTimer
    {
    public:
      // Computes the rate at which pulses come in over 'window_duration' in wall
      // time.
      explicit RateTimer(const common::Duration window_duration)
          : window_duration_(window_duration) {}
      ~RateTimer() {}

      RateTimer(const RateTimer &) = delete;
      RateTimer &operator=(const RateTimer &) = delete;

      // Returns the pulse rate in Hz.
      double ComputeRate() const
      {
        if (events_.empty())
        {
          return 0.;
        }
        return static_cast<double>(events_.size() - 1) /
               common::ToSeconds((events_.back().time - events_.front().time));
      }

      // Returns the ratio of the pulse rate (with supplied times) to the wall time
      // rate. For example, if a sensor produces pulses at 10 Hz, but we call Pulse
      // at 20 Hz wall time, this will return 2.
      double ComputeWallTimeRateRatio() const
      {
        if (events_.empty())
        {
          return 0.;
        }
        return common::ToSeconds((events_.back().time - events_.front().time)) /
               common::ToSeconds(events_.back().wall_time -
                                 events_.front().wall_time);
      }

      // Records an event that will contribute to the computed rate.
      void Pulse(common::Time time)
      {
        events_.push_back(Event{time, ClockType::now()});
        while (events_.size() > 2 &&
               (events_.back().wall_time - events_.front().wall_time) >
                   window_duration_)
        {
          events_.pop_front();
        }
      }

      // Returns a debug string representation.
      std::string DebugString() const
      {
        if (events_.size() < 2)
        {
          return "unknown";
        }
        std::ostringstream out;
        out << std::fixed << std::setprecision(2) << ComputeRate() << " Hz "
            << DeltasDebugString() << " (pulsed at "
            << ComputeWallTimeRateRatio() * 100. << "% real time)";
        return out.str();
      }

    private:
      struct Event
      {
        common::Time time;
        typename ClockType::time_point wall_time;
      };

      // Computes all differences in seconds between consecutive pulses.
      std::vector<double> ComputeDeltasInSeconds() const
      {
        CHECK_GT(events_.size(), 1);
        const size_t count = events_.size() - 1;
        std::vector<double> result;
        result.reserve(count);
        for (size_t i = 0; i != count; ++i)
        {
          result.push_back(
              common::ToSeconds(events_[i + 1].time - events_[i].time));
        }
        return result;
      }

      // Returns the average and standard deviation of the deltas.
      std::string DeltasDebugString() const
      {
        const auto deltas = ComputeDeltasInSeconds();
        const double sum = std::accumulate(deltas.begin(), deltas.end(), 0.);
        const double mean = sum / deltas.size();

        double squared_sum = 0.;
        for (const double x : deltas)
        {
          squared_sum += common::Pow2(x - mean);
        }
        const double sigma = std::sqrt(squared_sum / (deltas.size() - 1));

        std::ostringstream out;
        out << std::scientific << std::setprecision(2) << mean << " s +/- " << sigma
            << " s";
        return out.str();
      }

      std::deque<Event> events_;
      const common::Duration window_duration_;
    };

  } // namespace common
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_COMMON_RATE_TIMER_HPP_
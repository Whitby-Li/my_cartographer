//
// Created by whitby on 2025-05-03.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_WORK_QUEUE_HPP_
#define MY_CARTOGRAPHER_MAP_INTERNAL_WORK_QUEUE_HPP_

#include <chrono>
#include <deque>
#include <functional>

namespace my_cartographer
{
  namespace map
  {

    struct WorkItem
    {
      enum class Result
      {
        kDoNotRunOptimization,
        kRunOptimization,
      };

      std::chrono::steady_clock::time_point time;
      std::function<Result()> task;
    };

    using WorkQueue = std::deque<WorkItem>;

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_WORK_QUEUE_HPP_
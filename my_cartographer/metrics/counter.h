//
// Created by whitby on 2025-03-02.
//

#ifndef MY_CARTOGRAPHER_METRICS_COUNTER_H
#define MY_CARTOGRAPHER_METRICS_COUNTER_H

#include <map>
#include <vector>

namespace my_cartographer
{
  namespace metrics
  {

    class Counter
    {
    public:
      // Counter instance that does nothing. Safe for use in static initializers.
      static Counter *Null();

      virtual ~Counter() = default;
      virtual void Increment() = 0;
      virtual void Increment(double by_value) = 0;
    };

  } // namespace metrics
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_METRICS_COUNTER_H
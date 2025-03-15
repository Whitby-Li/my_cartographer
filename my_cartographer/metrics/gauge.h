//
// Created by whitby on 2025-03-02.
//

#ifndef MY_CARTOGRAPHER_METRICS_GAUGE_H
#define MY_CARTOGRAPHER_METRICS_GAUGE_H

#include <map>
#include <vector>

namespace my_cartographer
{
  namespace metrics
  {

    class Gauge
    {
    public:
      // Gauge instance that does nothing. Safe for use in static initializers.
      static Gauge *Null();

      virtual ~Gauge() = default;
      virtual void Increment() = 0;
      virtual void Increment(double by_value) = 0;
      virtual void Decrement() = 0;
      virtual void Decrement(double by_value) = 0;
      virtual void Set(double value) = 0;
    };

  } // namespace metrics
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_METRICS_GAUGE_H
//
// Created by whitby on 2025-03-02.
//

#ifndef MY_CARTOGRAPHER_METRICS_HISTOGRAM_H
#define MY_CARTOGRAPHER_METRICS_HISTOGRAM_H

#include <map>
#include <vector>

namespace my_cartographer
{
  namespace metrics
  {

    class Histogram
    {
    public:
      using BucketBoundaries = std::vector<double>;

      // Histogram instance that does nothing. Safe for use in static initializers.
      static Histogram *Null();

      static BucketBoundaries FixedWidth(double width, int num_finite_buckets);
      static BucketBoundaries ScaledPowersOf(double base, double scale_factor,
                                             double max_value);

      virtual ~Histogram() = default;
      virtual void Observe(double value) = 0;
    };

  } // namespace metrics
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_METRICS_HISTOGRAM_H
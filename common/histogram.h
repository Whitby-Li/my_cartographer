//
// Created by whitby on 2025-02-04.
//

#ifndef MY_CARTOGRAPHER_COMMON_HISTOGRAM_H_
#define MY_CARTOGRAPHER_COMMON_HISTOGRAM_H_

#include "port.hpp"

#include <string>
#include <vector>

namespace cartographer
{
  namespace common
  {
    class Histogram
    {
    public:
      void Add(float value);
      std::string ToString(int buckets) const;

    private:
      std::vector<float> values_;
    };
  } // namespace common
} // namespace cartographer

#endif // MY_CARTOGRAPHER_COMMON_HISTOGRAM_H_
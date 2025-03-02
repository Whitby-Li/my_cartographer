//
// Created by whitby on 2025-02-04.
//

#ifndef MY_CARTOGRAPHER_COMMON_HISTOGRAM_H_
#define MY_CARTOGRAPHER_COMMON_HISTOGRAM_H_

#include "my_cartographer/common/port.hpp"

#include <string>
#include <vector>

namespace my_cartographer
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
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_COMMON_HISTOGRAM_H_
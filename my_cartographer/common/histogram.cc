//
// Created by whitby on 2025-02-04.
//

#include "my_cartographer/common/histogram.h"

#include <absl/strings/str_cat.h>
#include <absl/strings/str_format.h>
#include <glog/logging.h>

#include <algorithm>
#include <numeric>
#include <string>

namespace my_cartographer
{
  namespace common
  {

    void Histogram::Add(const float value) { values_.push_back(value); }

    std::string Histogram::ToString(const int buckets) const
    {
      CHECK_GE(buckets, 1);
      if (values_.empty())
      {
        return "Count: 0";
      }
      const float min = *std::min_element(values_.begin(), values_.end());
      const float max = *std::max_element(values_.begin(), values_.end());
      const float mean =
          std::accumulate(values_.begin(), values_.end(), 0.f) / values_.size();
      std::string result = absl::StrCat("Count: ", values_.size(), "  Min: ", min,
                                        "  Max: ", max, "  Mean: ", mean);
      if (min == max)
      {
        return result;
      }
      CHECK_LT(min, max);
      float lower_bound = min;
      int total_count = 0;
      for (int i = 0; i != buckets; ++i)
      {
        const float upper_bound =
            (i + 1 == buckets)
                ? max
                : (max * (i + 1) / buckets + min * (buckets - i - 1) / buckets);
        int count = 0;
        for (const float value : values_)
        {
          if (lower_bound <= value &&
              (i + 1 == buckets ? value <= upper_bound : value < upper_bound))
          {
            ++count;
          }
        }
        total_count += count;
        absl::StrAppendFormat(&result, "\n[%f, %f%c", lower_bound, upper_bound,
                              i + 1 == buckets ? ']' : ')');
        constexpr int kMaxBarChars = 20;
        const int bar =
            (count * kMaxBarChars + values_.size() / 2) / values_.size();
        result += "\t";
        for (int i = 0; i != kMaxBarChars; ++i)
        {
          result += (i < (kMaxBarChars - bar)) ? " " : "#";
        }
        absl::StrAppend(&result, "\tCount: ", count, " (",
                        count * 1e2f / values_.size(), "%)",
                        "\tTotal: ", total_count, " (",
                        total_count * 1e2f / values_.size(), "%)");
        lower_bound = upper_bound;
      }
      return result;
    }

  } // namespace common
} // namespace my_cartographer
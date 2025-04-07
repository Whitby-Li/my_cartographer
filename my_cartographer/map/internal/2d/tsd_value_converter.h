//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_TSD_VALUE_CONVERTER_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_TSD_VALUE_CONVERTER_H

#include "my_cartographer/common/port.hpp"
#include "my_cartographer/common/math.hpp"
#include "my_cartographer/map/value_conversion_tables.h"

#include <glog/logging.h>

#include <cmath>
#include <vector>

namespace my_cartographer
{
  namespace map
  {

    // Provides conversions between float and uint16 representations for
    // truncated signed distance values and weights.
    class TSDValueConverter
    {
    public:
      TSDValueConverter(float max_tsd, float max_weight,
                        ValueConversionTables *conversion_tables);

      // Converts a tsd to a uint16 in the [1, 32767] range.
      inline uint16 TSDToValue(const float tsd) const
      {
        const int value =
            common::RoundToInt((ClampTSD(tsd) - min_tsd_) * tsd_resolution_) + 1;
        DCHECK_GE(value, 1);
        DCHECK_LE(value, 32767);
        return value;
      }

      // Converts a weight to a uint16 in the [1, 32767] range.
      inline uint16 WeightToValue(const float weight) const
      {
        const int value = common::RoundToInt((ClampWeight(weight) - min_weight_) *
                                             weight_resolution_) +
                          1;
        DCHECK_GE(value, 1);
        DCHECK_LE(value, 32767);
        return value;
      }

      // Converts a uint16 (which may or may not have the update marker set) to a
      // value in the range [min_tsd_, max_tsd_].
      inline float ValueToTSD(const uint16 value) const
      {
        return (*value_to_tsd_)[value];
      }

      // Converts a uint16 (which may or may not have the update marker set) to a
      // value in the range [min_weight_, max_weight_].
      inline float ValueToWeight(const uint16 value) const
      {
        return (*value_to_weight_)[value];
      }

      static uint16 getUnknownTSDValue() { return unknown_tsd_value_; }
      static uint16 getUnknownWeightValue() { return unknown_weight_value_; }
      static uint16 getUpdateMarker() { return update_marker_; }
      float getMaxTSD() const { return max_tsd_; }
      float getMinTSD() const { return min_tsd_; }
      float getMaxWeight() const { return max_weight_; }
      float getMinWeight() const { return min_weight_; }

    private:
      // Clamps TSD to be in the range [min_tsd_, max_tsd_].
      inline float ClampTSD(const float tsd) const
      {
        return common::Clamp(tsd, min_tsd_, max_tsd_);
      }
      // Clamps weight to be in the range [min_tsd_, max_tsd_].
      inline float ClampWeight(const float weight) const
      {
        return common::Clamp(weight, min_weight_, max_weight_);
      }

      float max_tsd_;
      float min_tsd_;
      float max_weight_;
      float tsd_resolution_;
      float weight_resolution_;
      static constexpr float min_weight_ = 0.f;
      static constexpr uint16 unknown_tsd_value_ = 0;
      static constexpr uint16 unknown_weight_value_ = 0;
      static constexpr uint16 update_marker_ = 1u << 15;

      const std::vector<float> *value_to_tsd_;
      const std::vector<float> *value_to_weight_;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_TSD_VALUE_CONVERTER_H
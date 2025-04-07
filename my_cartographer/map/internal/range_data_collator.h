//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_RANGE_DATA_COLLATOR_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_RANGE_DATA_COLLATOR_H

#include "my_cartographer/sensor/timed_point_cloud_data.h"

#include <absl/memory/memory.h>

#include <memory>

namespace my_cartographer
{
  namespace map
  {

    // Synchronizes TimedPointCloudData from different sensors. Input needs only be
    // monotonous in 'TimedPointCloudData::time', output is monotonous in per-point
    // timing. Up to one message per sensor is buffered, so a delay of the period of
    // the slowest sensor may be introduced, which can be alleviated by passing
    // subdivisions.
    class RangeDataCollator
    {
    public:
      explicit RangeDataCollator(
          const std::vector<std::string> &expected_range_sensor_ids)
          : expected_sensor_ids_(expected_range_sensor_ids.begin(),
                                 expected_range_sensor_ids.end()) {}

      // If timed_point_cloud_data has incomplete intensity data, we will fill the
      // missing intensities with kDefaultIntensityValue.
      sensor::TimedPointCloudOriginData AddRangeData(
          const std::string &sensor_id,
          sensor::TimedPointCloudData timed_point_cloud_data);

    private:
      sensor::TimedPointCloudOriginData CropAndMerge();

      const std::set<std::string> expected_sensor_ids_;
      // Store at most one message for each sensor.
      std::map<std::string, sensor::TimedPointCloudData> id_to_pending_data_;
      common::Time current_start_ = common::Time::min();
      common::Time current_end_ = common::Time::min();

      constexpr static float kDefaultIntensityValue = 0.f;
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_RANGE_DATA_COLLATOR_H
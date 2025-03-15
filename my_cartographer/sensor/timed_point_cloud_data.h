//
// Created by whitby on 2025-03-09.
//

#ifndef MY_CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H
#define MY_CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H

#include "my_cartographer/common/time.h"
#include "my_cartographer/sensor/point_cloud.h"

#include <Eigen/Core>

namespace my_cartographer
{
  namespace sensor
  {

    struct TimePointCloudData
    {
      common::Time time;
      Eigen::Vector3f origin;
      TimedPointCloud ranges;
      // 'intensities' has to be same size as 'ranges', or empty.
      std::vector<float> intensities;
    };

    struct TimedPointCloudOriginData
    {
      struct RangeMeasurement
      {
        TimedRangefinderPoint point_time;
        float intensity;
        size_t origin_index;
      };
      common::Time time;
      std::vector<Eigen::Vector3f> origins;
      std::vector<RangeMeasurement> ranges;
    };

    // Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
    proto::TimedPointCloudData ToProto(
        const TimedPointCloudData &timed_point_cloud_data);

    // Converts 'proto' to TimedPointCloudData.
    TimedPointCloudData FromProto(const proto::TimedPointCloudData &proto);

  } // namespace sensor
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H
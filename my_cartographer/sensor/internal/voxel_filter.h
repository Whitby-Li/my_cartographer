//
// Created by whitby on 2025-03-24.
//

#ifndef MY_CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
#define MY_CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/sensor/point_cloud.h"
#include "my_cartographer/sensor/timed_point_cloud_data.h"
#include "my_cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"

namespace my_cartographer
{
  namespace sensor
  {

    std::vector<RangefinderPoint> VoxelFilter(const std::vector<RangefinderPoint> &points, const float resolution);

    PointCloud VoxelFilter(const PointCloud &point_cloud, const float resolution);

    TimedPointCloudData VoxelFilter(const TimedPointCloudData &point_cloud_data, const float resolution);

    std::vector<TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(const std::vector<TimedPointCloudOriginData::RangeMeasurement> &range_measurements, const float resolution);

    proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(common::LuaParameterDictionary *const parameter_dictionary);

    PointCloud AdaptiveVoxelFilter(const PointCloud &point_cloud, const proto::AdaptiveVoxelFilterOptions &options);
  
  } // namespace sensor
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
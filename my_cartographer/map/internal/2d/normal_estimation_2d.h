//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_NORMAL_ESTIMATION_2D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_NORMAL_ESTIMATION_2D_H

#include "my_cartographer/map/proto/normal_estimation_options_2d.pb.h"
#include "my_cartographer/sensor/point_cloud.h"
#include "my_cartographer/sensor/range_data.h"
#include "my_cartographer/transform/transform.h"

namespace my_cartographer
{
  namespace map
  {

    proto::NormalEstimationOptions2D CreateNormalEstimationOptions2D(
        common::LuaParameterDictionary *parameter_dictionary);

    // Estimates the normal for each 'return' in 'range_data'.
    // Assumes the angles in the range data returns are sorted with respect to
    // the orientation of the vector from 'origin' to 'return'.
    std::vector<float> EstimateNormals(
        const sensor::RangeData &range_data,
        const proto::NormalEstimationOptions2D &normal_estimation_options);

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_NORMAL_ESTIMATION_2D_H
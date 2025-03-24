//
// Created by whitby on 2025-03-23.
//

#ifndef MY_CARTOGRAPHER_ODOMETRY_DATA_H
#define MY_CARTOGRAPHER_ODOMETRY_DATA_H

#include "my_cartographer/common/time.h"
#include "my_cartographer/transform/rigid_transform.h"
#include "my_cartographer/sensor/proto/sensor.pb.h"

namespace my_cartographer
{
  namespace sensor
  {

    struct OdometryData
    {
      common::Time time;
      transform::Rigid3d pose;
    };

    // Converts 'odometry_data' to a proto::OdometryData.
    proto::OdometryData ToProto(const OdometryData &odometry_data);

    // Converts 'proto' to a OdometryData.
    OdometryData FromProto(const proto::OdometryData &proto);

  } // namespace sensor
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_ODOMETRY_DATA_H
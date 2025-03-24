//
// Created by whitby on 2025-03-23.
//

#include "my_cartographer/sensor/odometry_data.h"
#include "my_cartographer/transform/transform.h"

namespace my_cartographer
{
  namespace sensor
  {

    proto::OdometryData ToProto(const OdometryData &odometry_data)
    {
      proto::OdometryData proto;
      proto.set_timestamp(common::ToUniversal(odometry_data.time));
      *proto.mutable_pose() = transform::ToProto(odometry_data.pose);
      return proto;
    }

    OdometryData FromProto(const proto::OdometryData &proto)
    {
      return OdometryData{common::FromUniversal(proto.timestamp()),
                          transform::ToRigid3(proto.pose())};
    }

  } // namespace sensor
} // namespace my_cartographer
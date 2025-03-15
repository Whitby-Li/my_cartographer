//
// Created by whitby on 2025-03-08.
//

#include "my_cartographer/sensor/fixed_frame_pose_data.h"

namespace my_cartographer
{
    namespace sensor
    {
        
        proto::FixedFramePoseData ToProto(const FixedFramePoseData& pose_data) {
            proto::FixedFramePoseData proto;
            proto.set_timestamp(common::ToUniversal(pose_data.time));
            if (pose_data.pose.has_value()) {
              *proto.mutable_pose() = transform::ToProto(pose_data.pose.value());
            }
            return proto;
          }
          
          FixedFramePoseData FromProto(const proto::FixedFramePoseData& proto) {
            return FixedFramePoseData{common::FromUniversal(proto.timestamp()),
                                      proto.has_pose()
                                          ? absl::optional<transform::Rigid3d>(
                                                transform::ToRigid3(proto.pose()))
                                          : absl::optional<transform::Rigid3d>()};
          }

    } // namespace sensor
} // namespace my_cartographer
//
// Created by whitby on 2025-03-08.
//

#ifndef MY_CARTOGRAPHER_SENSOR_FIXED_FRAME_POSE_DATA_H
#define MY_CARTOGRAPHER_SENSOR_FIXED_FRAME_POSE_DATA_H

#include "my_cartographer/common/time.h"
#include "my_cartographer/sensor/proto/sensor.pb.h"
#include "my_cartographer/transform/rigid_transform.h"

#include <absl/types/optional.h>

#include <memory>

namespace my_cartographer
{
    namespace sensor
    {

        // The fixed frame pose data (like GPS, pose, etc.) will be used in the optimization
        struct FixedFramePoseData
        {
            common::Time time;
            absl::optional<transform::Rigid3d> pose;
        }

        // Converts 'pose_data' to a proto::FixedFramePoseData.
        proto::FixedFramePoseData ToProto(const FixedFramePoseData& pose_data);

        // Converts 'proto' to a FixedFramePoseData.
        FixedFramePoseData FromProto(const proto::FixedFramePoseData& proto);

    } // namespace sensor
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_SENSOR_FIXED_FRAME_POSE_DATA_H
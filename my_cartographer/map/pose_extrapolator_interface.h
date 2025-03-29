//
// Created by whitby on 2025-03-26.
//

#ifndef MY_CARTOGRAPHER_MAP_POSE_EXTRAPOLATOR_INTERFACE_H
#define MY_CARTOGRAPHER_MAP_POSE_EXTRAPOLATOR_INTERFACE_H

#include "my_cartographer/common/time.h"
#include "my_cartographer/transform/rigid_transform.h"
#include "my_cartographer/transform/timestamped_transform.h"
#include "my_cartographer/sensor/imu_data.h"
#include "my_cartographer/sensor/odometry_data.h"
#include "my_cartographer/map/proto/pose_extrapolator_options.pb.h"

#include <memory>
#include <tuple>

namespace my_cartographer
{
  namespace map
  {

    proto::PoseExtrapolatorOptions CreatePoseExtrapolatorOptions(
        common::LuaParameterDictionary *const parameter_dictionary);

    class PoseExtrapolatorInterface
    {
    public:
      struct ExtrapolationResult
      {
        // The poses for the requested times at index 0 to N-1.
        std::vector<transform::Rigid3f> previous_poses;
        // The pose for the requested time at index N.
        transform::Rigid3d current_pose;
        Eigen::Vector3d current_velocity;
        Eigen::Quaterniond gravity_from_tracking;
      };

      PoseExtrapolatorInterface(const PoseExtrapolatorInterface &) = delete;
      PoseExtrapolatorInterface &operator=(const PoseExtrapolatorInterface &) = delete;
      virtual ~PoseExtrapolatorInterface() {}

      // // TODO: Remove dependency cycle.
      // static std::unique_ptr<PoseExtrapolatorInterface> CreateWithImuData(
      //     const proto::PoseExtrapolatorOptions &options,
      //     const std::vector<sensor::ImuData> &imu_data,
      //     const std::vector<transform::TimestampedTransform> &initial_poses);

      // Return the time of the last added pose or Time::min() if no pose was added yet.
      virtual common::Time GetLastPoseTime() const = 0;
      virtual common::Time GetLastExtrapolatedTime() const = 0;

      virtual void AddPose(common::Time time, const transform::Rigid3d &pose) = 0;
      virtual void AddImuData(const sensor::ImuData &imu_data) = 0;
      virtual void AddOdometryData(const sensor::OdometryData &odometry_data) = 0;
      virtual transform::Rigid3d ExtrapolatePose(common::Time time) = 0;

      virtual ExtrapolationResult ExtrapolatePosesWithGravity(const std::vector<common::Time> &times) = 0;

      // Returns the current gravity alignment estimate as a rotation from the tracking frame into a gravity aligned frame.
      virtual Eigen::Quaterniond EstimateGravityOrientation(common::Time time) = 0;

    protected:
      PoseExtrapolatorInterface() {}
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_POSE_EXTRAPOLATOR_INTERFACE_H
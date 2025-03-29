//
// Created by whitby on 2025-03-26.
//

#ifndef MY_CARTOGRAPHER_MAP_POSE_EXTRAPOLATOR_H
#define MY_CARTOGRAPHER_MAP_POSE_EXTRAPOLATOR_H

#include "my_cartographer/map/pose_extrapolator_interface.h"
#include "my_cartographer/sensor/imu_data.h"
#include "my_cartographer/sensor/odometry_data.h"
#include "my_cartographer/transform/rigid_transform.h"
#include "my_cartographer/map/imu_tracker.h"

#include <deque>
#include <memory>

namespace my_cartographer
{
  namespace map
  {

    // Keep poses for a certain duration to estimate linear and angular velocity.
    // Uses the velocities to extrapolate motion.
    // Uses IMU and/or odometry data if available to improve the extrapolation.
    class PoseExtrapolator : public PoseExtrapolatorInterface
    {
    public:
      explicit PoseExtrapolator(common::Duration pose_queue_duration, double imu_gravity_time_constant);

      PoseExtrapolator(const PoseExtrapolator &) = delete;
      PoseExtrapolator &operator=(const PoseExtrapolator &) = delete;

      static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
          common::Duration pose_queue_duration, double imu_gravity_time_constant,
          const sensor::ImuData &imu_data);

      // Returns the time of the last added pose or Time::min() if no pose was added yet.
      common::Time GetLastPoseTime() const override;
      common::Time GetLastExtrapolatedTime() const override;

      void AddPose(common::Time time, const transform::Rigid3d &pose) override;
      void AddImuData(const sensor::ImuData &imu_data) override;
      void AddOdometryData(const sensor::OdometryData &odometry_data) override;
      transform::Rigid3d ExtrapolatePose(common::Time time) override;

      ExtrapolationResult ExtrapolatePosesWithGravity(
          const std::vector<common::Time> &times) override;

      // Returns the current gravity alignment estimate as a rotation from the tracking frame into a gravity aligned frame.
      Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

    private:
      void UpdateVelocitiesFromPoses();
      void TrimImuData();
      void TrimOdometryData();
      void AdvanceImuTracker(common::Time time, ImuTracker *imu_tracker) const;
      Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                             ImuTracker *imu_tracker) const;
      Eigen::Vector3d ExtrapolateTranslation(common::Time time);

      const common::Duration pose_queue_duration_;
      struct TimedPose
      {
        common::Time time;
        transform::Rigid3d pose;
      };
      std::deque<TimedPose> timed_pose_queue_;
      Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
      Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

      const double gravity_time_constant_;
      std::deque<sensor::ImuData> imu_data_;
      std::unique_ptr<ImuTracker> imu_tracker_;
      std::unique_ptr<ImuTracker> odometry_imu_tracker_;
      std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
      TimedPose cached_extrapolated_pose_;

      std::deque<sensor::OdometryData> odometry_data_;
      Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
      Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
    };

  } // namespace map
} // namespace my_cartographer

#endif
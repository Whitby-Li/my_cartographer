//
// Created by whitby on 2025-03-26.
//

#ifndef MY_CARTOGRAPHER_MAP_IMU_TRACKER_H
#define MY_CARTOGRAPHER_MAP_IMU_TRACKER_H

#include "my_cartographer/common/time.h"

#include <Eigen/Geometry>

namespace my_cartographer
{
  namespace map
  {

    // Keeps track of the orientation using angular velocities and linear accelerations from an IMU.
    // Because average linear acceleration (assuming slow movement) is a direct measurement of gravity, roll/pitch does not drift, though yaw does.
    class ImuTracker
    {
    public:
      ImuTracker(double imu_gravity_time_constant, common::Time time);

      /**
       * @brief 旋转运动估计，并更新当前旋转姿态下重力方向
       */
      void Advance(common::Time time);

      /**
       * @brief 添加 IMU 加速度观测数据，并更新重力方向，并调整旋转估计
       * @note 最好是用静止或匀速直线运动情况，加速度计数据最接近重力方向
       */
      void AddImuLinearAccelerationObservation(const Eigen::Vector3d &imu_linear_acceleration);

      /**
       * @brief 更新 IMU 角速度
       */
      void AddImuAngularVelocityObservation(const Eigen::Vector3d &imu_angular_velocity);

      // Query the current time.
      common::Time time() const { return time_; }

      // Query the current orientation estimate.
      Eigen::Quaterniond orientation() const { return orientation_; }

    private:
      const double imu_gravity_time_constant_;
      common::Time time_;
      common::Time last_linear_acceleration_time_;
      Eigen::Quaterniond orientation_;
      Eigen::Vector3d gravity_vector_;
      Eigen::Vector3d imu_angular_velocity_;
    };

  }
}

#endif // MY_CARTOGRAPHER_MAP_IMU_TRACKER_H
//
// Created by whitby on 2025-03-08.
//

#ifndef MY_CARTOGRAPHER_SENSOR_IMU_DATA_H
#define MY_CARTOGRAPHER_SENSOR_IMU_DATA_H

namespace my_cartographer
{
  namespace sensor
  {

    struct ImuData
    {
      common::Time time;
      Eigen::Vector3d linear_acceleration;
      Eigen::Vector3d angular_velocity;
    };

    // Converts 'imu_data' to a proto::ImuData.
    proto::ImuData ToProto(const ImuData &imu_data);

    // Converts 'proto' to a ImuData.
    ImuData FromProto(const proto::ImuData &proto);

  } // namespace sensor
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_SENSOR_IMU_DATA_H
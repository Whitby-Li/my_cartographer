//
// Created by whitby on 2025-05-04.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
#define MY_CARTOGRAPHER_MAP_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "my_cartographer/common/internal/rate_timer.hpp"
#include "my_cartographer/common/port.hpp"
#include "my_cartographer/map/internal/local_slam_result_data.h"
#include "my_cartographer/map/submaps.h"
#include "my_cartographer/map/trajectory_builder_interface.h"
#include "my_cartographer/map/collator_interface.h"
#include "my_cartographer/map/dispatchable.hpp"

namespace my_cartographer
{
  namespace map
  {

    // Collates sensor data using a sensor::CollatorInterface, then passes it on to
    // a mapping::TrajectoryBuilderInterface which is common for 2D and 3D.
    class CollatedTrajectoryBuilder : public TrajectoryBuilderInterface
    {
    public:
      using SensorId = TrajectoryBuilderInterface::SensorId;

      CollatedTrajectoryBuilder(
          const proto::TrajectoryBuilderOptions &trajectory_options,
          sensor::CollatorInterface *sensor_collator, int trajectory_id,
          const std::set<SensorId> &expected_sensor_ids,
          std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder);
      ~CollatedTrajectoryBuilder() override {}

      CollatedTrajectoryBuilder(const CollatedTrajectoryBuilder &) = delete;
      CollatedTrajectoryBuilder &operator=(const CollatedTrajectoryBuilder &) =
          delete;

      void AddSensorData(
          const std::string &sensor_id,
          const sensor::TimedPointCloudData &timed_point_cloud_data) override
      {
        AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
      }

      void AddSensorData(const std::string &sensor_id,
                         const sensor::ImuData &imu_data) override
      {
        AddData(sensor::MakeDispatchable(sensor_id, imu_data));
      }

      void AddSensorData(const std::string &sensor_id,
                         const sensor::OdometryData &odometry_data) override
      {
        AddData(sensor::MakeDispatchable(sensor_id, odometry_data));
      }

      void AddSensorData(
          const std::string &sensor_id,
          const sensor::FixedFramePoseData &fixed_frame_pose_data) override
      {
        if (collate_fixed_frame_)
        {
          AddData(sensor::MakeDispatchable(sensor_id, fixed_frame_pose_data));
          return;
        }
        wrapped_trajectory_builder_->AddSensorData(sensor_id,
                                                   fixed_frame_pose_data);
      }

      void AddSensorData(const std::string &sensor_id,
                         const sensor::LandmarkData &landmark_data) override
      {
        if (collate_landmarks_)
        {
          AddData(sensor::MakeDispatchable(sensor_id, landmark_data));
          return;
        }
        wrapped_trajectory_builder_->AddSensorData(sensor_id, landmark_data);
      }

      void AddLocalSlamResultData(std::unique_ptr<map::LocalSlamResultData>
                                      local_slam_result_data) override
      {
        AddData(std::move(local_slam_result_data));
      }

    private:
      void AddData(std::unique_ptr<sensor::Data> data);

      void HandleCollatedSensorData(const std::string &sensor_id,
                                    std::unique_ptr<sensor::Data> data);

      sensor::CollatorInterface *const sensor_collator_;
      const bool collate_landmarks_;
      const bool collate_fixed_frame_;
      const int trajectory_id_;
      std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder_;

      // Time at which we last logged the rates of incoming sensor data.
      std::chrono::steady_clock::time_point last_logging_time_;
      std::map<std::string, common::RateTimer<>> rate_timers_;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
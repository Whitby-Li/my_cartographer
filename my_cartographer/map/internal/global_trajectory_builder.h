//
// Created by whitby on 2025-05-04.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_
#define MY_CARTOGRAPHER_MAP_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "my_cartographer/map/internal/2d/local_trajectory_builder_2d.h"
#include "my_cartographer/map/internal/2d/pose_graph_2d.h"
#include "my_cartographer/map/internal/3d/local_trajectory_builder_3d.h"
#include "my_cartographer/map/internal/3d/pose_graph_3d.h"
#include "my_cartographer/map/internal/local_slam_result_data.h"
#include "my_cartographer/map/trajectory_builder_interface.h"
#include "my_cartographer/metrics/family_factory.hpp"

namespace my_cartographer
{
  namespace map
  {

    std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
        std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
        const int trajectory_id, PoseGraph2D *const pose_graph,
        const TrajectoryBuilderInterface::LocalSlamResultCallback &
            local_slam_result_callback,
        const absl::optional<MotionFilter> &pose_graph_odometry_motion_filter);

    std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
        std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder,
        const int trajectory_id, PoseGraph3D *const pose_graph,
        const TrajectoryBuilderInterface::LocalSlamResultCallback &
            local_slam_result_callback,
        const absl::optional<MotionFilter> &pose_graph_odometry_motion_filter);

    void GlobalTrajectoryBuilderRegisterMetrics(
        metrics::FamilyFactory *family_factory);

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_
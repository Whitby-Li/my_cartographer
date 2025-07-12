//
// Created by whitby on 2025-06-08.
//

#include "my_cartographer/metrics/register.h"

#include "my_cartographer/map/internal/2d/local_trajectory_builder_2d.h"
#include "my_cartographer/map/internal/2d/pose_graph_2d.h"
#include "my_cartographer/map/internal/3d/local_trajectory_builder_3d.h"
#include "my_cartographer/map/internal/3d/pose_graph_3d.h"
#include "my_cartographer/map/internal/constraints/constraint_builder_2d.h"
#include "my_cartographer/map/internal/constraints/constraint_builder_3d.h"
#include "my_cartographer/map/internal/global_trajectory_builder.h"
#include "my_cartographer/map/trajectory_collator.h"

namespace my_cartographer
{
  namespace metrics
  {

    void RegisterAllMetrics(FamilyFactory *registry)
    {
      map::constraints::ConstraintBuilder2D::RegisterMetrics(registry);
      map::constraints::ConstraintBuilder3D::RegisterMetrics(registry);
      map::GlobalTrajectoryBuilderRegisterMetrics(registry);
      map::LocalTrajectoryBuilder2D::RegisterMetrics(registry);
      map::LocalTrajectoryBuilder3D::RegisterMetrics(registry);
      map::PoseGraph2D::RegisterMetrics(registry);
      map::PoseGraph3D::RegisterMetrics(registry);
      sensor::TrajectoryCollator::RegisterMetrics(registry);
    }

  } // namespace metrics
} // namespace my_cartographer
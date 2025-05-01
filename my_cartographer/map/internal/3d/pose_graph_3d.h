//
// Created by whitby on 2025-04-26.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_3D_POSE_GRAPH_3D_H_
#define MY_CARTOGRAPHER_MAP_INTERNAL_3D_POSE_GRAPH_3D_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <absl/container/flat_hash_map.h>
#include <absl/synchronization/mutex.h>

#include "my_cartographer/common/fixed_ratio_sampler.h"
#include "my_cartographer/common/thread_pool.h"
#include "my_cartographer/common/time.h"
#include "my_cartographer/map/3d/submap_3d.h"
#include "my_cartographer/map/internal/constraints/constraint_builder.h"
#include "my_cartographer/map/internal/optimization/optimization_problem_3d.h"
#include "my_cartographer/map/internal/trajectory_connectivity_state.h"
#include "my_cartographer/map/internal/pose_graph_data.h"
#include "my_cartographer/map/pose_graph.h"
#include "my_cartographer/map/pose_graph_trimmer.h"
#include "my_cartographer/metrics/family_factory.hpp"
#include "my_cartographer/sensor/fixed_frame_pose_data.h"
#include "my_cartographer/sensor/landmark_data.h"
#include "my_cartographer/sensor/odometry_data.h"
#include "my_cartographer/sensor/point_cloud.h"
#include "my_cartographer/transform/rigid_transform.h"

namespace my_cartographer
{
  namespace map
  {

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_3D_POSE_GRAPH_3D_H_
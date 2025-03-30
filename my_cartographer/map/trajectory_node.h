//
// Created by whitby on 2025-03-25.
//

#ifndef MY_CARTOGRAPHER_MAP_TRAJECTORY_NODE_H
#define MY_CARTOGRAPHER_MAP_TRAJECTORY_NODE_H

#include "my_cartographer/common/time.h"
#include "my_cartographer/sensor/range_data.h"
#include "my_cartographer/transform/rigid_transform.h"
#include "my_cartographer/map/proto/trajectory_node_data.pb.h"

#include <absl/types/optional.h>
#include <Eigen/Core>

#include <memory>
#include <vector>

namespace my_cartographer
{
  namespace map
  {

    struct TrajectoryNodePose
    {

      struct ConstantPoseData
      {
        common::Time time;
        transform::Rigid3d local_pose;
      };

      // The nose pose in the global SLAM frame.
      transform::Rigid3d global_pose;

      absl::optional<ConstantPoseData> constant_pose_data;
    };

    struct TrajectoryNode
    {
      struct Data
      {
        common::Time time;

        // Transform to approximately gravity align the tracking frame as determined by local SLAM.
        Eigen::Quaterniond gravity_alignment;

        // Used for loop closure in 2D: voxel filtered returns in 'gravity_alignment' frame.
        sensor::PointCloud filtered_gravity_aligned_point_cloud;

        // Used for loop closure in 3D.
        sensor::PointCloud high_resolution_point_cloud;
        sensor::PointCloud low_resolution_point_cloud;
        Eigen::VectorXf rotational_scan_matcher_histogram;

        // The node pose in the local SLAM frame.
        transform::Rigid3d local_pose;
      };

      common::Time time() const { return constant_data->time; }

      // This must be a shared_ptr.
      // If the data is used for visualization while the node is being trimmed, it must survive until all use finishes.
      std::shared_ptr<const Data> constant_data;

      // The node pose in the global SLAM frame.
      transform::Rigid3d global_pose;
    };

    proto::TrajectoryNodeData ToProto(const TrajectoryNode::Data &constant_data);
    TrajectoryNode::Data FromProto(const proto::TrajectoryNodeData &proto);

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_TRAJECTORY_NODE_H
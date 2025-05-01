//
// Created by whitby on 2025-04-26.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_
#define MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_

#include <map>
#include <set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "my_cartographer/common/time.h"
#include "my_cartographer/map/id.hpp"
#include "my_cartographer/map/pose_graph_interface.h"
#include "my_cartographer/sensor/fixed_frame_pose_data.h"
#include "my_cartographer/sensor/imu_data.h"
#include "my_cartographer/sensor/map_by_time.hpp"
#include "my_cartographer/sensor/odometry_data.h"

namespace my_cartographer
{
  namespace map
  {
    namespace optimization
    {

      // Implements the SPA loop closure optimization problem.
      template <typename NodeDataType, typename SubmapDataType,
                typename RigidTransformType>
      class OptimizationProblemInterface
      {
      public:
        using Constraint = PoseGraphInterface::Constraint;
        using LandmarkNode = PoseGraphInterface::LandmarkNode;

        OptimizationProblemInterface() {}
        virtual ~OptimizationProblemInterface() {}

        OptimizationProblemInterface(const OptimizationProblemInterface &) = delete;
        OptimizationProblemInterface &operator=(const OptimizationProblemInterface &) =
            delete;

        virtual void AddImuData(int trajectory_id,
                                const sensor::ImuData &imu_data) = 0;
        virtual void AddOdometryData(int trajectory_id,
                                     const sensor::OdometryData &odometry_data) = 0;
        virtual void AddTrajectoryNode(int trajectory_id,
                                       const NodeDataType &node_data) = 0;
        virtual void InsertTrajectoryNode(const NodeId &node_id,
                                          const NodeDataType &node_data) = 0;
        virtual void TrimTrajectoryNode(const NodeId &node_id) = 0;
        virtual void AddSubmap(int trajectory_id,
                               const RigidTransformType &global_submap_pose) = 0;
        virtual void InsertSubmap(const SubmapId &submap_id,
                                  const RigidTransformType &global_submap_pose) = 0;
        virtual void TrimSubmap(const SubmapId &submap_id) = 0;
        virtual void SetMaxNumIterations(int32 max_num_iterations) = 0;

        // Optimizes the global poses.
        virtual void Solve(
            const std::vector<Constraint> &constraints,
            const std::map<int, PoseGraphInterface::TrajectoryState> &
                trajectories_state,
            const std::map<std::string, LandmarkNode> &landmark_nodes) = 0;

        virtual const MapById<NodeId, NodeDataType> &node_data() const = 0;
        virtual const MapById<SubmapId, SubmapDataType> &submap_data() const = 0;
        virtual const std::map<std::string, transform::Rigid3d> &landmark_data()
            const = 0;
        virtual const sensor::MapByTime<sensor::ImuData> &imu_data() const = 0;
        virtual const sensor::MapByTime<sensor::OdometryData> &odometry_data()
            const = 0;
      };

    } // namespace optimization
  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_INTERFACE_H_
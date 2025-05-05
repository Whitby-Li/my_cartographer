//
// Created by whitby on 2025-05-03.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_POSE_GRAPH_DATA_HPP_
#define MY_CARTOGRAPHER_MAP_INTERNAL_POSE_GRAPH_DATA_HPP_

#include <map>
#include <set>
#include <vector>

#include "my_cartographer/map/internal/optimization/optimization_problem_2d.h"
#include "my_cartographer/map/internal/optimization/optimization_problem_3d.h"
#include "my_cartographer/map/internal/trajectory_connectivity_state.h"
#include "my_cartographer/map/pose_graph.h"
#include "my_cartographer/map/pose_graph_interface.h"
#include "my_cartographer/map/submaps.h"

namespace my_cartographer
{
  namespace map
  {

    // The current state of the submap in the background threads. After this
    // transitions to 'kFinished', all nodes are tried to match
    // against this submap. Likewise, all new nodes are matched against submaps in
    // that state.
    enum class SubmapState
    {
      kNoConstraintSearch,
      kFinished
    };

    struct InternalTrajectoryState
    {
      enum class DeletionState
      {
        NORMAL,
        SCHEDULED_FOR_DELETION,
        WAIT_FOR_DELETION
      };

      PoseGraphInterface::TrajectoryState state =
          PoseGraphInterface::TrajectoryState::ACTIVE;
      DeletionState deletion_state = DeletionState::NORMAL;
    };

    struct InternalSubmapData
    {
      std::shared_ptr<const Submap> submap;
      SubmapState state = SubmapState::kNoConstraintSearch;

      // IDs of the nodes that were inserted into this map together with
      // constraints for them. They are not to be matched again when this submap
      // becomes 'kFinished'.
      std::set<NodeId> node_ids;
    };

    struct PoseGraphData
    {
      // Submaps get assigned an ID and state as soon as they are seen, even
      // before they take part in the background computations.
      MapById<SubmapId, InternalSubmapData> submap_data;

      // Global submap poses currently used for displaying data.
      MapById<SubmapId, optimization::SubmapSpec2D> global_submap_poses_2d;
      MapById<SubmapId, optimization::SubmapSpec3D> global_submap_poses_3d;

      // Data that are currently being shown.
      MapById<NodeId, TrajectoryNode> trajectory_nodes;

      // Global landmark poses with all observations.
      std::map<std::string /* landmark ID */, PoseGraphInterface::LandmarkNode>
          landmark_nodes;

      // How our various trajectories are related.
      TrajectoryConnectivityState trajectory_connectivity_state;
      int num_trajectory_nodes = 0;
      std::map<int, InternalTrajectoryState> trajectories_state;

      // Set of all initial trajectory poses.
      std::map<int, PoseGraph::InitialTrajectoryPose> initial_trajectory_poses;

      std::vector<PoseGraphInterface::Constraint> constraints;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_POSE_GRAPH_DATA_HPP_
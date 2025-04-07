//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_POSE_GRAPH_TRIMMER_H
#define MY_CARTOGRAPHER_MAP_POSE_GRAPH_TRIMMER_H

#include "my_cartographer/map/id.hpp"
#include "my_cartographer/map/pose_graph_interface.h"

namespace my_cartographer
{
  namespace map
  {

    // Implemented by the pose graph to provide thread-safe access to functions for
    // trimming the graph.
    class Trimmable
    {
    public:
      virtual ~Trimmable() {}

      virtual int num_submaps(int trajectory_id) const = 0;

      virtual std::vector<SubmapId> GetSubmapIds(int trajectory_id) const = 0;
      // Returns finished submaps with optimized poses only.
      virtual MapById<SubmapId, PoseGraphInterface::SubmapData>
      GetOptimizedSubmapData() const = 0;
      virtual const MapById<NodeId, TrajectoryNode> &GetTrajectoryNodes() const = 0;
      virtual const std::vector<PoseGraphInterface::Constraint> &GetConstraints()
          const = 0;

      // Trim 'submap_id' and corresponding intra-submap nodes. They
      // will no longer take part in scan matching, loop closure, visualization.
      // The numbering remains unchanged.
      virtual void TrimSubmap(const SubmapId &submap_id) = 0;

      // Checks if the given trajectory is finished or not.
      virtual bool IsFinished(int trajectory_id) const = 0;

      // Sets the state for a specific trajectory.
      virtual void SetTrajectoryState(
          int trajectory_id, PoseGraphInterface::TrajectoryState state) = 0;
    };

    // An interface to implement algorithms that choose how to trim the pose graph.
    class PoseGraphTrimmer
    {
    public:
      virtual ~PoseGraphTrimmer() {}

      // Called once after each pose graph optimization.
      virtual void Trim(Trimmable *pose_graph) = 0;

      // Checks if this trimmer is in a terminatable state.
      virtual bool IsFinished() = 0;
    };

    // Keeps the last 'num_submaps_to_keep' of the trajectory with 'trajectory_id'
    // to implement localization without mapping.
    class PureLocalizationTrimmer : public PoseGraphTrimmer
    {
    public:
      PureLocalizationTrimmer(int trajectory_id, int num_submaps_to_keep);
      ~PureLocalizationTrimmer() override {}

      void Trim(Trimmable *pose_graph) override;
      bool IsFinished() override;

    private:
      const int trajectory_id_;
      int num_submaps_to_keep_;
      bool finished_ = false;
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_POSE_GRAPH_TRIMMER_H
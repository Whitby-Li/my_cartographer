//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_POSE_GRAPH_H
#define MY_CARTOGRAPHER_MAP_POSE_GRAPH_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/map/id.hpp"
#include "my_cartographer/map/pose_graph_interface.h"
#include "my_cartographer/map/pose_graph_trimmer.h"
#include "my_cartographer/map/proto/pose_graph.pb.h"
#include "my_cartographer/map/proto/pose_graph_options.pb.h"
#include "my_cartographer/map/proto/serialization.pb.h"
#include "my_cartographer/map/submaps.h"
#include "my_cartographer/map/trajectory_node.h"
#include "my_cartographer/sensor/fixed_frame_pose_data.h"
#include "my_cartographer/sensor/imu_data.h"
#include "my_cartographer/sensor/odometry_data.h"
#include "my_cartographer/sensor/landmark_data.h"
#include "my_cartographer/sensor/map_by_time.hpp"

#include <absl/container/flat_hash_map.h>

#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace my_cartographer
{
  namespace map
  {

    proto::PoseGraphOptions CreatePoseGraphOptions(
        common::LuaParameterDictionary *const parameter_dictionary);

    class PoseGraph : public PoseGraphInterface
    {
    public:
      struct InitialTrajectoryPose
      {
        int to_trajectory_id;
        transform::Rigid3d relative_pose;
        common::Time time;
      };

      PoseGraph() {}
      ~PoseGraph() override {}

      PoseGraph(const PoseGraph &) = delete;
      PoseGraph &operator=(const PoseGraph &) = delete;

      // Inserts an IMU measurement.
      virtual void AddImuData(int trajectory_id,
                              const sensor::ImuData &imu_data) = 0;

      // Inserts an odometry measurement.
      virtual void AddOdometryData(int trajectory_id,
                                   const sensor::OdometryData &odometry_data) = 0;

      // Inserts a fixed frame pose measurement.
      virtual void AddFixedFramePoseData(
          int trajectory_id,
          const sensor::FixedFramePoseData &fixed_frame_pose_data) = 0;

      // Inserts landmarks observations.
      virtual void AddLandmarkData(int trajectory_id,
                                   const sensor::LandmarkData &landmark_data) = 0;

      // Finishes the given trajectory.
      virtual void FinishTrajectory(int trajectory_id) = 0;

      // Freezes a trajectory. Poses in this trajectory will not be optimized.
      virtual void FreezeTrajectory(int trajectory_id) = 0;

      // Adds a 'submap' from a proto with the given 'global_pose' to the
      // appropriate trajectory.
      virtual void AddSubmapFromProto(const transform::Rigid3d &global_pose,
                                      const proto::Submap &submap) = 0;

      // Adds a 'node' from a proto with the given 'global_pose' to the
      // appropriate trajectory.
      virtual void AddNodeFromProto(const transform::Rigid3d &global_pose,
                                    const proto::Node &node) = 0;

      // Sets the trajectory data from a proto.
      virtual void SetTrajectoryDataFromProto(
          const proto::TrajectoryData &data) = 0;

      // Adds information that 'node_id' was inserted into 'submap_id'. The submap
      // has to be deserialized first.
      virtual void AddNodeToSubmap(const NodeId &node_id,
                                   const SubmapId &submap_id) = 0;

      // Adds serialized constraints. The corresponding trajectory nodes and submaps
      // have to be deserialized before calling this function.
      virtual void AddSerializedConstraints(
          const std::vector<Constraint> &constraints) = 0;

      // Adds a 'trimmer'. It will be used after all data added before it has been
      // included in the pose graph.
      virtual void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) = 0;

      // Gets the current trajectory clusters.
      virtual std::vector<std::vector<int>> GetConnectedTrajectories() const = 0;

      proto::PoseGraph ToProto(bool include_unfinished_submaps) const override;

      // Returns the IMU data.
      virtual sensor::MapByTime<sensor::ImuData> GetImuData() const = 0;

      // Returns the odometry data.
      virtual sensor::MapByTime<sensor::OdometryData> GetOdometryData() const = 0;

      // Returns the fixed frame pose data.
      virtual sensor::MapByTime<sensor::FixedFramePoseData> GetFixedFramePoseData()
          const = 0;

      // Returns the landmark data.
      virtual std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
      GetLandmarkNodes() const = 0;

      // Sets a relative initial pose 'relative_pose' for 'from_trajectory_id' with
      // respect to 'to_trajectory_id' at time 'time'.
      virtual void SetInitialTrajectoryPose(int from_trajectory_id,
                                            int to_trajectory_id,
                                            const transform::Rigid3d &pose,
                                            const common::Time time) = 0;
    };

    std::vector<PoseGraph::Constraint> FromProto(
        const ::google::protobuf::RepeatedPtrField<proto::PoseGraph::Constraint> &constraint_protos);

    proto::PoseGraph::Constraint ToProto(const PoseGraph::Constraint &constraint);

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_POSE_GRAPH_H

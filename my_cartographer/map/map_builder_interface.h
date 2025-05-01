//
// Created by whitby on 2025-04-26.
//

#ifndef MY_CARTOGRAPHER_MAP_MAP_BUILDER_INTERFACE_H_
#define MY_CARTOGRAPHER_MAP_MAP_BUILDER_INTERFACE_H_

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/common/port.hpp"
#include "my_cartographer/io/proto_stream_interface.h"
#include "my_cartographer/map/id.hpp"
#include "my_cartographer/map/pose_graph_interface.h"
#include "my_cartographer/map/proto/map_builder_options.pb.h"
#include "my_cartographer/map/proto/trajectory_builder_options.pb.h"
#include "my_cartographer/map/submaps.h"
#include "my_cartographer/map/trajectory_builder_interface.h"

#include <Eigen/Geometry>

#include <set>
#include <string>
#include <vector>

namespace my_cartographer
{
  namespace map
  {

    proto::MapBuilderOptions CreateMapBuilderOptions(
        common::LuaParameterDictionary *const parameter_dictionary);

    /**
     * @note This interface is used for both library and RPC implementations.
     * Implementations wire up the complete SLAM stack.
     */
    class MapBuilderInterface
    {
    public:
      using LocalSlamResultCallback = TrajectoryBuilderInterface::LocalSlamResultCallback;
      using SensorId = TrajectoryBuilderInterface::SensorId;

      MapBuilderInterface() {}
      virtual ~MapBuilderInterface() {}

      MapBuilderInterface(const MapBuilderInterface &) = delete;
      MapBuilderInterface &operator=(const MapBuilderInterface &) = delete;

      // Creates a new trajectory builder and returns its index.
      virtual int AddTrajectoryBuilder(
          const std::set<SensorId> &expected_sensor_ids,
          const proto::TrajectoryBuilderOptions &trajectory_options,
          LocalSlamResultCallback local_slam_result_callback) = 0;

      // Creates a new trajectory and returns its index. 
      // Querying the trajectory builder for it will return 'nullptr'.
      virtual int AddTrajectoryForDeserialization(
          const proto::TrajectoryBuilderOptionsWithSensorIds &
              options_with_sensor_ids_proto) = 0;

      // Returns the 'TrajectoryBuilderInterface' corresponding to the specified
      // 'trajectory_id' or 'nullptr' if the trajectory has no corresponding
      // builder.
      virtual TrajectoryBuilderInterface *GetTrajectoryBuilder(
          int trajectory_id) const = 0;

      // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
      // i.e. no further sensor data is expected.
      virtual void FinishTrajectory(int trajectory_id) = 0;

      // Fills the SubmapQuery::Response corresponding to 'submap_id'. Returns an
      // error string on failure, or an empty string on success.
      virtual std::string SubmapToProto(const SubmapId &submap_id,
                                        proto::SubmapQuery::Response *response) = 0;

      // Serializes the current state to a proto stream. If
      // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
      // submaps that have not yet received all rangefinder data insertions, will
      // be included in the serialized state.
      virtual void SerializeState(bool include_unfinished_submaps,
                                  io::ProtoStreamWriterInterface *writer) = 0;

      // Serializes the current state to a proto stream file on the host system. If
      // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
      // submaps that have not yet received all rangefinder data insertions, will
      // be included in the serialized state.
      // Returns true if the file was successfully written.
      virtual bool SerializeStateToFile(bool include_unfinished_submaps,
                                        const std::string &filename) = 0;

      // Loads the SLAM state from a proto stream. Returns the remapping of new
      // trajectory_ids.
      virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
      LoadState(io::ProtoStreamReaderInterface *reader, bool load_frozen_state) = 0;

      // Loads the SLAM state from a pbstream file. Returns the remapping of new
      // trajectory_ids.
      virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
      LoadStateFromFile(const std::string &filename, bool load_frozen_state) = 0;

      virtual int num_trajectory_builders() const = 0;

      virtual PoseGraphInterface *pose_graph() = 0;

      virtual const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds> &
      GetAllTrajectoryBuilderOptions() const = 0;
    };

    
  }  // namespace map
}  // namespace my_cartographer

#endif  // MY_CARTOGRAPHER_MAP_MAP_BUILDER_INTERFACE_H_

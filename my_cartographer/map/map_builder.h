//
// Created by whitby on 2025-04-26.
//

#ifndef MY_CARTOGRAPHER_MAP_MAP_BUILDER_H_
#define MY_CARTOGRAPHER_MAP_MAP_BUILDER_H_

#include <memory>

#include "my_cartographer/common/thread_pool.h"
#include "my_cartographer/map/map_builder_interface.h"
#include "my_cartographer/map/pose_graph.h"
#include "my_cartographer/map/proto/map_builder_options.pb.h"
#include "my_cartographer/map/collator_interface.h"

namespace my_cartographer
{
  namespace map
  {

    // Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps) and a PoseGraph for loop closure.
    class MapBuilder : public MapBuilderInterface
    {
    public:
      explicit MapBuilder(const proto::MapBuilderOptions &options);
      ~MapBuilder() override {}

      MapBuilder(const MapBuilder &) = delete;
      MapBuilder &operator=(const MapBuilder &) = delete;

      int AddTrajectoryBuilder(
          const std::set<SensorId> &expected_sensor_ids,
          const proto::TrajectoryBuilderOptions &trajectory_options,
          LocalSlamResultCallback local_slam_result_callback) override;

      int AddTrajectoryForDeserialization(
          const proto::TrajectoryBuilderOptionsWithSensorIds
              &options_with_sensor_ids_proto) override;

      void FinishTrajectory(int trajectory_id) override;

      std::string SubmapToProto(const SubmapId &submap_id,
                                proto::SubmapQuery::Response *response) override;

      void SerializeState(bool include_unfinished_submaps,
                          io::ProtoStreamWriterInterface *writer) override;

      bool SerializeStateToFile(bool include_unfinished_submaps,
                                const std::string &filename) override;

      std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                                   bool load_frozen_state) override;

      std::map<int, int> LoadStateFromFile(const std::string &filename,
                                           const bool load_frozen_state) override;

      PoseGraphInterface *pose_graph() override
      {
        return pose_graph_.get();
      }

      int num_trajectory_builders() const override
      {
        return trajectory_builders_.size();
      }

      TrajectoryBuilderInterface *GetTrajectoryBuilder(
          int trajectory_id) const override
      {
        return trajectory_builders_.at(trajectory_id).get();
      }

      const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
          &GetAllTrajectoryBuilderOptions() const override
      {
        return all_trajectory_builder_options_;
      }

    private:
      const proto::MapBuilderOptions options_;
      common::ThreadPool thread_pool_;

      std::unique_ptr<PoseGraph> pose_graph_;

      std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
      std::vector<std::unique_ptr<TrajectoryBuilderInterface>>
          trajectory_builders_;
      std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
          all_trajectory_builder_options_;
    };

    std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
        const proto::MapBuilderOptions &options);

  }  // namespace map
}  // namespace my_cartographer

#endif  // MY_CARTOGRAPHER_MAP_MAP_BUILDER_H_
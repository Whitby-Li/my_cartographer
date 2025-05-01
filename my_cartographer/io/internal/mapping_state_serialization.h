//
// Created by whitby on 2025-04-26.
//

#ifndef MY_CARTOGRAPHER_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_
#define MY_CARTOGRAPHER_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_

#include "my_cartographer/io/proto_stream_interface.h"
#include "my_cartographer/map/pose_graph.h"
#include "my_cartographer/map/proto/trajectory_builder_options.pb.h"

namespace my_cartographer
{
  namespace io
  {

    // The current serialization format version.
    static constexpr int kMappingStateSerializationFormatVersion = 2;
    static constexpr int kFormatVersionWithoutSubmapHistograms = 1;

    // Serialize mapping state to a pbstream.
    void WritePbStream(
        const map::PoseGraph &pose_graph,
        const std::vector<map::proto::TrajectoryBuilderOptionsWithSensorIds> &
            builder_options,
        ProtoStreamWriterInterface *const writer, bool include_unfinished_submaps);

  } // namespace io
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_
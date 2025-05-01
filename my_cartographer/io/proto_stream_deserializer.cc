//
// Created by whitby on 2025-04-26.
//

#include "my_cartographer/io/proto_stream_deserializer.h"

#include <glog/logging.h>

#include "my_cartographer/io/internal/mapping_state_serialization.h"
#include "my_cartographer/io/proto_stream.h"

namespace my_cartographer
{
  namespace io
  {

    namespace
    {

      map::proto::SerializationHeader ReadHeaderOrDie(
          ProtoStreamReaderInterface *const reader)
      {
        map::proto::SerializationHeader header;
        CHECK(reader->ReadProto(&header)) << "Failed to read SerializationHeader.";
        return header;
      }

      bool IsVersionSupported(const map::proto::SerializationHeader &header)
      {
        return header.format_version() == kMappingStateSerializationFormatVersion ||
               header.format_version() == kFormatVersionWithoutSubmapHistograms;
      }

    } // namespace

    map::proto::PoseGraph DeserializePoseGraphFromFile(
        const std::string &file_name)
    {
      ProtoStreamReader reader(file_name);
      ProtoStreamDeserializer deserializer(&reader);
      return deserializer.pose_graph();
    }

    ProtoStreamDeserializer::ProtoStreamDeserializer(
        ProtoStreamReaderInterface *const reader)
        : reader_(reader), header_(ReadHeaderOrDie(reader))
    {
      CHECK(IsVersionSupported(header_)) << "Unsupported serialization format \""
                                         << header_.format_version() << "\"";

      CHECK(ReadNextSerializedData(&pose_graph_))
          << "Serialized stream misses PoseGraph.";
      CHECK(pose_graph_.has_pose_graph())
          << "Serialized stream order corrupt. Expecting `PoseGraph` after "
             "`SerializationHeader`, but got field tag "
          << pose_graph_.data_case();

      CHECK(ReadNextSerializedData(&all_trajectory_builder_options_))
          << "Serialized stream misses `AllTrajectoryBuilderOptions`.";
      CHECK(all_trajectory_builder_options_.has_all_trajectory_builder_options())
          << "Serialized stream order corrupt. Expecting "
             "`AllTrajectoryBuilderOptions` after "
             "PoseGraph, got field tag "
          << all_trajectory_builder_options_.data_case();

      CHECK_EQ(pose_graph_.pose_graph().trajectory_size(),
               all_trajectory_builder_options_.all_trajectory_builder_options()
                   .options_with_sensor_ids_size());
    }

    bool ProtoStreamDeserializer::ReadNextSerializedData(
        map::proto::SerializedData *data)
    {
      return reader_->ReadProto(data);
    }

  } // namespace io
} // namespace my_cartographer
//
// Created by whitby on 2025-04-26.
//

#ifndef MY_CARTOGRAPHER_IO_PROTO_STREAM_DESERIALIZER_H_
#define MY_CARTOGRAPHER_IO_PROTO_STREAM_DESERIALIZER_H_

#include "my_cartographer/io/proto_stream_interface.h"
#include "my_cartographer/map/proto/pose_graph.pb.h"
#include "my_cartographer/map/proto/serialization.pb.h"
#include "my_cartographer/map/proto/trajectory_builder_options.pb.h"

namespace my_cartographer
{
  namespace io
  {

    // Helper function for deserializing the PoseGraph from a proto stream file.
    map::proto::PoseGraph DeserializePoseGraphFromFile(const std::string &file_name);

    // Helper for deserializing a previously serialized mapping state from a
    // proto stream, abstracting away the format parsing logic.
    class ProtoStreamDeserializer
    {
    public:
      explicit ProtoStreamDeserializer(ProtoStreamReaderInterface *const reader);

      ProtoStreamDeserializer(const ProtoStreamDeserializer &) = delete;
      ProtoStreamDeserializer &operator=(const ProtoStreamDeserializer &) = delete;
      ProtoStreamDeserializer(ProtoStreamDeserializer &&) = delete;

      map::proto::SerializationHeader &header() { return header_; }

      map::proto::PoseGraph &pose_graph()
      {
        return *pose_graph_.mutable_pose_graph();
      }
      const map::proto::PoseGraph &pose_graph() const
      {
        return pose_graph_.pose_graph();
      }

      const map::proto::AllTrajectoryBuilderOptions &
      all_trajectory_builder_options()
      {
        return all_trajectory_builder_options_.all_trajectory_builder_options();
      }

      // Reads the next `SerializedData` message of the ProtoStream into `data`.
      // Returns `true` if the message was successfully read or `false` in case
      // there are no-more messages or an error occurred.
      bool ReadNextSerializedData(map::proto::SerializedData *data);

    private:
      ProtoStreamReaderInterface *reader_;

      map::proto::SerializationHeader header_;
      map::proto::SerializedData pose_graph_;
      map::proto::SerializedData all_trajectory_builder_options_;
    };

  } // namespace io
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_IO_PROTO_STREAM_DESERIALIZER_H_
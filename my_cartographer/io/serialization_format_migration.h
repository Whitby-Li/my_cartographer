//
// Created by whitby on 2025-04-26.
//

#ifndef MY_CARTOGRAPHER_IO_SERIALIZATION_FORMAT_MIGRATION_H_
#define MY_CARTOGRAPHER_IO_SERIALIZATION_FORMAT_MIGRATION_H_

#include "my_cartographer/io/proto_stream_interface.h"
#include "my_cartographer/map/id.hpp"
#include "my_cartographer/map/proto/serialization.pb.h"

namespace my_cartographer
{
  namespace io
  {

    // This helper function migrates the input stream, which is supposed
    // to contain submaps without histograms (stream format version 1) to
    // an output stream containing submaps with histograms (version 2).
    void MigrateStreamVersion1ToVersion2(
        ProtoStreamReaderInterface *const input,
        ProtoStreamWriterInterface *const output,
        bool include_unfinished_submaps);

    map::MapById<map::SubmapId, map::proto::Submap>
    MigrateSubmapFormatVersion1ToVersion2(
        const map::MapById<map::SubmapId, map::proto::Submap> &
            submap_id_to_submaps,
        map::MapById<map::NodeId, map::proto::Node> &node_id_to_nodes,
        const map::proto::PoseGraph &pose_graph_proto);

  } // namespace io
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_IO_SERIALIZATION_FORMAT_MIGRATION_H_
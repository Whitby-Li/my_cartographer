//
// Created by whitby on 2025-04-26.
//

#ifndef MY_CARTOGRAPHER_IO_PROTO_STREAM_H_
#define MY_CARTOGRAPHER_IO_PROTO_STREAM_H_

#include "my_cartographer/io/proto_stream_interface.h"

#include <fstream>

namespace my_cartographer
{
  namespace io
  {

    // A simple writer of a compressed sequence of protocol buffer messages to a
    // file. The format is not intended to be compatible with any other format used
    // outside of Cartographer.
    //
    // TODO(whess): Compress the file instead of individual messages for better
    // compression performance? Should we use LZ4?
    class ProtoStreamWriter : public ProtoStreamWriterInterface
    {
    public:
      ProtoStreamWriter(const std::string &filename);
      ~ProtoStreamWriter() = default;

      ProtoStreamWriter(const ProtoStreamWriter &) = delete;
      ProtoStreamWriter &operator=(const ProtoStreamWriter &) = delete;

      void WriteProto(const google::protobuf::Message &proto) override;
      bool Close() override;

    private:
      void Write(const std::string &uncompressed_data);

      std::ofstream out_;
    };

    // A reader of the format produced by ProtoStreamWriter.
    class ProtoStreamReader : public ProtoStreamReaderInterface
    {
    public:
      explicit ProtoStreamReader(const std::string &filename);
      ~ProtoStreamReader() = default;

      ProtoStreamReader(const ProtoStreamReader &) = delete;
      ProtoStreamReader &operator=(const ProtoStreamReader &) = delete;

      bool ReadProto(google::protobuf::Message *proto) override;
      bool eof() const override;

    private:
      bool Read(std::string *decompressed_data);

      std::ifstream in_;
    };

  }  // namespace io
}  // namespace my_cartographer

#endif  // MY_CARTOGRAPHER_IO_PROTO_STREAM_H_
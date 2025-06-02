//
// Created by whitby on 2025-06-02.
//

#include "my_cartographer/io/file_writer.h"

namespace my_cartographer
{
  namespace io
  {

    StreamFileWriter::StreamFileWriter(const std::string &filename)
        : filename_(filename), out_(filename, std::ios::out | std::ios::binary) {}

    StreamFileWriter::~StreamFileWriter() {}

    bool StreamFileWriter::Write(const char* const data, const size_t len)
    {
      if (out_.bad())
      {
        return false;
      }

      out_.write(data, len);
      return !out_.bad();
    }

    bool StreamFileWriter::Close()
    {
      if (out_.bad())
      {
        return false;
      }
      out_.close();
      return !out_.bad();
    }

    bool StreamFileWriter::WriteHeader(const char *const data, const size_t len)
    {
      if (out_.bad())
      {
        return false;
      }
      out_.flush();
      out_.seekp(0);
      return Write(data, len);
    }

    std::string StreamFileWriter::GetFilename() { return filename_; }

  } // namespace io
} // namespace my_cartographer
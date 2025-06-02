//
// Created by whitby on 2025-06-02.
//

#pragma once

#include <fstream>
#include <functional>
#include <memory>

#include "my_cartographer/common/port.hpp"

namespace my_cartographer
{
  namespace io
  {

    // Simple abstraction for a file.
    class FileWriter
    {
    public:
      FileWriter() {}
      FileWriter(const FileWriter &) = delete;
      FileWriter &operator=(const FileWriter &) = delete;

      virtual ~FileWriter() {}

      // Write 'data' to the beginning of the file. This is required to overwrite
      // fixed size headers which contain the number of points once we actually know
      // how many points there are.
      virtual bool WriteHeader(const char *data, size_t len) = 0;

      virtual bool Write(const char *data, size_t len) = 0;
      virtual bool Close() = 0;
      virtual std::string GetFilename() = 0;
    };

    // An Implementation of file using std::ofstream.
    class StreamFileWriter : public FileWriter
    {
    public:
      ~StreamFileWriter() override;

      StreamFileWriter(const std::string &filename);

      bool Write(const char *data, size_t len) override;
      bool WriteHeader(const char *data, size_t len) override;
      bool Close() override;
      std::string GetFilename() override;

    private:
      const std::string filename_;
      std::ofstream out_;
    };

    using FileWriterFactory = std::function<std::unique_ptr<FileWriter>(const std::string &filename)>;

  } // namespace io
} // namespace my_cartographer
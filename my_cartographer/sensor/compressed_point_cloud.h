//
// Created by whitby on 2025-03-09.
//

#ifndef MY_CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H
#define MY_CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H

#include "my_cartographer/common/port.hpp"
#include "my_cartographer/sensor/proto/sensor.pb.h"
#include "my_cartographer/sensor/point_cloud.h"

#include <Eigen/Core>

#include <vector>
#include <iterator>

namespace my_cartographer
{
  namespace sensor
  {
    // A compressed representation of a point cloud consisting a collection of points (Vector3d) without time information.
    // Internally, points are grouped by blocks.
    // Each block encodes a bit of meta data (number of points in block, coordinates of the first point in block) and encodes each point with a fixed bit rate in relation to the block.
    class CompressedPointCloud
    {
    public:
      class ConstIterator;

      CompressedPointCloud() : num_points_(0) {}
      explicit CompressedPointCloud(const PointCloud &point_cloud);
      explicit CompressedPointCloud(const proto::CompressedPointCloud &proto);

      // Returns decompressed point cloud.
      PointCloud Decompress() const;

      bool empty() const;
      size_t size() const;
      ConstIterator begin() const;
      ConstIterator end() const;

      bool operator==(const CompressedPointCloud &right_hand_container) const;
      proto::CompressedPointCloud ToProto() const;

    private:
      std::vector<int32> point_data_;
      size_t num_points_;
    };

    // Forward iterator for compressed point clouds.
    class CompressedPointCloud::ConstIterator
    {
    public:
      using iterator_category = std::forward_iterator_tag;
      using value_type = RangefinderPoint;
      using difference_type = int64;
      using pointer = const RangefinderPoint *;
      using reference = const RangefinderPoint &;

      // Creates begin iterator.
      explicit ConstIterator(const CompressedPointCloud *compressed_point_cloud);

      // Creates end iterator.
      static ConstIterator EndIterator(const CompressedPointCloud *compressed_point_cloud);

      RangefinderPoint operator*() const;
      ConstIterator &operator++();
      bool operator!=(const ConstIterator &it) const;

    private:
      // Reads next point from buffer. Also handles reading the meta data of the
      // next block, if the current block is depleted.
      void ReadNextPoint();

      const CompressedPointCloud *compressed_point_cloud_;
      size_t remaining_points_;
      int32 remaining_points_in_current_block_;
      Eigen::Vector3f current_point_;
      Eigen::Vector3i current_block_coordinates_;
      std::vector<int32>::const_iterator input_;
    };

  } // namespace sensor
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H
//
// Created by whitby on 2025-03-09.
//

#include "my_cartographer/sensor/point_cloud.h"
#include "my_cartographer/transform/transform.h"

namespace my_cartographer
{
  namespace sensor
  {

    PointCloud::PointCloud() {}

    PointCloud::PointCloud(const std::vector<PointType> &points) : points_(points) {}

    PointCloud::PointCloud(const std::vector<PointType> &points, const std::vector<float> &intensities)
        : points_(points), intensities_(intensities)
    {
      CHECK_EQ(points_.size(), intensities_.size());
    }

    size_t PointCloud::size() const { return points_.size(); }

    bool PointCloud::empty() const { return points_.empty(); }

    const std::vector<PointCloud::PointType> &PointCloud::points() const { return points_; }

    const std::vector<float> &PointCloud::intensities() const { return intensities_; }

    const PointCloud::PointType &PointCloud::operator[](const size_t index) const
    {
      CHECK_LT(index, size());
      return points_[index];
    }

    PointCloud::ConstIterator PointCloud::begin() const { return points_.begin(); }

    PointCloud::ConstIterator PointCloud::end() const { return points_.end(); }

    void PointCloud::push_back(PointType value)
    {
      points_.push_back(std::move(value));
    }

    PointCloud TransformPointCloud(const PointCloud &point_cloud, const transform::Rigid3f &transform)
    {
      std::vector<RangefinderPoint> points;
      points.reserve(point_cloud.size());
      for (const auto &point : point_cloud)
      {
        points.push_back(transform * point);
      }
      return PointCloud(points, point_cloud.intensities());
    }

    TimedPointCloud TransformTimedPointCloud(const TimedPointCloud &point_cloud, const transform::Rigid3f &transform)
    {
      TimedPointCloud result;
      result.reserve(point_cloud.size());
      for (const TimedRangefinderPoint &point : point_cloud)
      {
        result.push_back(transform * point);
      }
      return result;
    }

    PointCloud CropPointCloud(const PointCloud &point_cloud, const float min_z, const float max_z)
    {
      return point_cloud.copy_if([min_z, max_z](const RangefinderPoint &point)
                                 { return min_z <= point.position.z() && point.position.z() <= max_z; });
    }

  }
}
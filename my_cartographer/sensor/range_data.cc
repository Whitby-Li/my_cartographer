//
// Created by whitby on 2025-03-23.
//

#include "my_cartographer/sensor/range_data.h"
#include "my_cartographer/transform/transform.h"

namespace my_cartographer
{
  namespace sensor
  {

    RangeData TransformRangeData(const RangeData &range_data, const transform::Rigid3f &transform)
    {
      return RangeData{transform * range_data.origin,
                       TransformPointCloud(range_data.returns, transform),
                       TransformPointCloud(range_data.misses, transform)};
    }

    RangeData CropRangeData(const RangeData &range_data, const float min_z, const float max_z)
    {
      return RangeData{range_data.origin,
                       CropPointCloud(range_data.returns, min_z, max_z),
                       CropPointCloud(range_data.misses, min_z, max_z)};
    }

    proto::RangeData ToProto(const RangeData &range_data)
    {
      proto::RangeData proto;
      *proto.mutable_origin() = transform::ToProto(range_data.origin);
      proto.mutable_returns()->Reserve(range_data.returns.size());
      for (const RangefinderPoint &point : range_data.returns)
      {
        *proto.add_returns() = ToProto(point);
      }
      proto.mutable_misses()->Reserve(range_data.misses.size());
      for (const RangefinderPoint &point : range_data.misses)
      {
        *proto.add_misses() = ToProto(point);
      }
      return proto;
    }

    RangeData FromProto(const proto::RangeData &proto)
    {
      std::vector<RangefinderPoint> returns;
      if (proto.returns_size() > 0)
      {
        returns.reserve(proto.returns().size());
        for (const auto &point_proto : proto.returns())
        {
          returns.push_back(FromProto(point_proto));
        }
      }
      else
      {
        returns.reserve(proto.returns_legacy().size());
        for (const auto &point_proto : proto.returns_legacy())
        {
          returns.push_back({transform::ToEigen(point_proto)});
        }
      }
      std::vector<RangefinderPoint> misses;
      if (proto.misses_size() > 0)
      {
        misses.reserve(proto.misses().size());
        for (const auto &point_proto : proto.misses())
        {
          misses.push_back(FromProto(point_proto));
        }
      }
      else
      {
        misses.reserve(proto.misses_legacy().size());
        for (const auto &point_proto : proto.misses_legacy())
        {
          misses.push_back({transform::ToEigen(point_proto)});
        }
      }
      return RangeData{transform::ToEigen(proto.origin()), PointCloud(returns),
                       PointCloud(misses)};
    }

  } // namespace sensor
} // namespace my_cartographer

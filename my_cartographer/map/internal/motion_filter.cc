//
// Created by whitby on 2025-04-06.
//

#include "my_cartographer/map/internal/motion_filter.h"
#include "my_cartographer/transform/transform.h"

#include <glog/logging.h>

namespace my_cartographer
{
  namespace map
  {

    proto::MotionFilterOptions CreateMotionFilterOptions(
        common::LuaParameterDictionary *const parameter_dictionary)
    {
      proto::MotionFilterOptions options;
      options.set_max_time_seconds(
          parameter_dictionary->GetDouble("max_time_seconds"));
      options.set_max_distance_meters(
          parameter_dictionary->GetDouble("max_distance_meters"));
      options.set_max_angle_radians(
          parameter_dictionary->GetDouble("max_angle_radians"));
      return options;
    }

    MotionFilter::MotionFilter(const proto::MotionFilterOptions &options)
        : options_(options) {}

    bool MotionFilter::IsSimilar(const common::Time time,
                                 const transform::Rigid3d &pose)
    {
      LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
          << "Motion filter reduced the number of nodes to "
          << 100. * num_different_ / num_total_ << "%.";
      ++num_total_;
      if (num_total_ > 1 &&
          time - last_time_ <= common::FromSeconds(options_.max_time_seconds()) &&
          (pose.translation() - last_pose_.translation()).norm() <=
              options_.max_distance_meters() &&
          transform::GetAngle(pose.inverse() * last_pose_) <=
              options_.max_angle_radians())
      {
        return true;
      }
      last_time_ = time;
      last_pose_ = pose;
      ++num_different_;
      return false;
    }

  } // namespace map
} // namespace my_cartographer
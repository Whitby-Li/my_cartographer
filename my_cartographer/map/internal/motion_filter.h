//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_MOTION_FILTER_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_MOTION_FILTER_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/common/time.h"
#include "my_cartographer/map/proto/motion_filter_options.pb.h"
#include "my_cartographer/transform/rigid_transform.h"

#include <limits>

namespace my_cartographer
{
  namespace map
  {

    proto::MotionFilterOptions CreateMotionFilterOptions(
        common::LuaParameterDictionary *parameter_dictionary);

    // Takes poses as input and filters them to get fewer poses.
    class MotionFilter
    {
    public:
      explicit MotionFilter(const proto::MotionFilterOptions &options);

      // If the accumulated motion (linear, rotational, or time) is above the
      // threshold, returns false. Otherwise the relative motion is accumulated and
      // true is returned.
      bool IsSimilar(common::Time time, const transform::Rigid3d &pose);

    private:
      const proto::MotionFilterOptions options_;
      int num_total_ = 0;
      int num_different_ = 0;
      common::Time last_time_;
      transform::Rigid3d last_pose_;
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_MOTION_FILTER_H
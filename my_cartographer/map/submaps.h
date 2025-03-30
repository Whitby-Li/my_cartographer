//
// Created by whitby on 2025-03-29.
//

#ifndef MY_CARTOGRAPHER_MAP_SUBMAPS_H
#define MY_CARTOGRAPHER_MAP_SUBMAPS_H

#include "my_cartographer/common/math.hpp"
#include "my_cartographer/common/port.hpp"
#include "my_cartographer/map/id.hpp"
#include "my_cartographer/map/probability_values.h"
#include "my_cartographer/map/proto/serialization.pb.h"
#include "my_cartographer/map/proto/submap_visualization.pb.h"
#include "my_cartographer/map/trajectory_node.h"

#include <glog/logging.h>

#include <memory>
#include <vector>

namespace my_cartographer
{
  namespace map
  {

    // Converts the given probability to log odds.
    inline float Logit(float probability)
    {
      return std::log(probability / (1.f - probability));
    }

    const float kMaxLogOdds = Logit(kMaxProbability);
    const float kMinLogOdds = Logit(kMinProbability);

    // Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
    // kMaxLogOdds] is mapped to [1, 255].
    inline uint8 ProbabilityToLogOddsInteger(const float probability)
    {
      const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                           254.f / (kMaxLogOdds - kMinLogOdds)) +
                        1;
      CHECK_LE(1, value);
      CHECK_GE(255, value);
      return value;
    }

    // An individual submap, which has a 'local_pose' in the local map frame,
    // keeps track of how many range data were inserted into it, 
    // and sets 'insertion_finished' when the map no longer changes and is ready for loop closing.
    class Submap
    {
    public:
      Submap(const transform::Rigid3d &local_submap_pose)
          : local_pose_(local_submap_pose) {}
      virtual ~Submap() {}

      virtual proto::Submap ToProto(bool include_grid_data) const = 0;
      virtual void UpdateFromProto(const proto::Submap &proto) = 0;

      // Fills data into the 'response'.
      virtual void ToResponseProto(
          const transform::Rigid3d &global_submap_pose,
          proto::SubmapQuery::Response *response) const = 0;

      // Pose of this submap in the local map frame.
      transform::Rigid3d local_pose() const { return local_pose_; }

      // Number of RangeData inserted.
      int num_range_data() const { return num_range_data_; }
      void set_num_range_data(const int num_range_data)
      {
        num_range_data_ = num_range_data;
      }

      bool insertion_finished() const { return insertion_finished_; }
      void set_insertion_finished(bool insertion_finished)
      {
        insertion_finished_ = insertion_finished;
      }

    private:
      const transform::Rigid3d local_pose_;
      int num_range_data_ = 0;
      bool insertion_finished_ = false;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_SUBMAPS_H
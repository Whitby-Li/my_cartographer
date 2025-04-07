//
// Created by whitby on 2025-04-06.
//

#include "my_cartographer/map/pose_graph_trimmer.h"

#include <glog/logging.h>

namespace my_cartographer
{
  namespace map
  {

    PureLocalizationTrimmer::PureLocalizationTrimmer(const int trajectory_id,
                                                     const int num_submaps_to_keep)
        : trajectory_id_(trajectory_id), num_submaps_to_keep_(num_submaps_to_keep)
    {
      CHECK_GE(num_submaps_to_keep, 2) << "Cannot trim with less than 2 submaps";
    }

    void PureLocalizationTrimmer::Trim(Trimmable *const pose_graph)
    {
      if (pose_graph->IsFinished(trajectory_id_))
      {
        num_submaps_to_keep_ = 0;
      }

      auto submap_ids = pose_graph->GetSubmapIds(trajectory_id_);
      for (std::size_t i = 0; i + num_submaps_to_keep_ < submap_ids.size(); ++i)
      {
        pose_graph->TrimSubmap(submap_ids.at(i));
      }

      if (num_submaps_to_keep_ == 0)
      {
        finished_ = true;
        pose_graph->SetTrajectoryState(
            trajectory_id_, PoseGraphInterface::TrajectoryState::DELETED);
      }
    }

    bool PureLocalizationTrimmer::IsFinished() { return finished_; }

  } // namespace map
}  // namespace my_cartographer
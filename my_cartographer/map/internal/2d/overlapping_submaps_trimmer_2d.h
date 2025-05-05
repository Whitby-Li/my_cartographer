//
// Created by whitby on 2025-05-04.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_OVERLAPPING_SUBMAPS_TRIMMER_2D_H_
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_OVERLAPPING_SUBMAPS_TRIMMER_2D_H_

#include "my_cartographer/common/port.hpp"
#include "my_cartographer/map/pose_graph_trimmer.h"

namespace my_cartographer
{
  namespace map
  {

    // Trims submaps that have less than 'min_covered_cells_count' cells not
    // overlapped by at least 'fresh_submaps_count` submaps.
    class OverlappingSubmapsTrimmer2D : public PoseGraphTrimmer
    {
    public:
      OverlappingSubmapsTrimmer2D(uint16 fresh_submaps_count,
                                  double min_covered_area,
                                  uint16 min_added_submaps_count)
          : fresh_submaps_count_(fresh_submaps_count),
            min_covered_area_(min_covered_area),
            min_added_submaps_count_(min_added_submaps_count) {}
      ~OverlappingSubmapsTrimmer2D() override = default;

      void Trim(Trimmable *pose_graph) override;
      bool IsFinished() override { return finished_; }

    private:
      // Number of the most recent submaps to keep.
      const uint16 fresh_submaps_count_;
      // Minimum area of covered space to keep submap from trimming measured in m^2.
      const double min_covered_area_;
      // Number of added submaps before the trimmer is invoked.
      const uint16 min_added_submaps_count_;
      // Current finished submap count.
      uint16 current_submap_count_ = 0;

      bool finished_ = false;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_OVERLAPPING_SUBMAPS_TRIMMER_2D_H_
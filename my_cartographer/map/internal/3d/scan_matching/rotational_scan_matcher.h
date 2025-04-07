//
// Created by whitby on 2025-04-07.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_ROTATIONAL_SCAN_MATCHER_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_ROTATIONAL_SCAN_MATCHER_H

#include "my_cartographer/sensor/point_cloud.h"

#include <Eigen/Geometry>

#include <vector>

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      class RotationalScanMatcher
      {
      public:
        // Rotates the given 'histogram' by the given 'angle'. This might lead to
        // rotations of a fractional bucket which is handled by linearly
        // interpolating.
        static Eigen::VectorXf RotateHistogram(const Eigen::VectorXf &histogram,
                                               float angle);

        // Computes the histogram for a gravity aligned 'point_cloud'.
        static Eigen::VectorXf ComputeHistogram(const sensor::PointCloud &point_cloud,
                                                int histogram_size);

        explicit RotationalScanMatcher(const Eigen::VectorXf *histogram);

        // Scores how well 'histogram' rotated by 'initial_angle' can be understood as
        // further rotated by certain 'angles' relative to the 'nodes'. Each angle
        // results in a score between 0 (worst) and 1 (best).
        std::vector<float> Match(const Eigen::VectorXf &histogram,
                                 float initial_angle,
                                 const std::vector<float> &angles) const;

      private:
        const Eigen::VectorXf *histogram_;
      };

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_ROTATIONAL_SCAN_MATCHER_H
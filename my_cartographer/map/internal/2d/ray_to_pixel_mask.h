//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_RAY_TO_PIXEL_MASK_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_RAY_TO_PIXEL_MASK_H

#include "my_cartographer/transform/transform.h"

#include <vector>

namespace my_cartographer
{
  namespace map
  {

    // Compute all pixels that contain some part of the line segment connecting
    // 'scaled_begin' and 'scaled_end'. 'scaled_begin' and 'scaled_end' are scaled
    // by 'subpixel_scale'. 'scaled_begin' and 'scaled_end' are expected to be
    // greater than zero. Return values are in pixels and not scaled.
    std::vector<Eigen::Array2i> RayToPixelMask(const Eigen::Array2i &scaled_begin,
                                               const Eigen::Array2i &scaled_end,
                                               int subpixel_scale);

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_RAY_TO_PIXEL_MASK_H
//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_2D_XY_INDEX_H
#define MY_CARTOGRAPHER_MAP_2D_XY_INDEX_H

#include "my_cartographer/common/math.hpp"
#include "my_cartographer/common/port.hpp"
#include "my_cartographer/map/proto/cell_limits_2d.pb.h"

#include <Eigen/Core>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>

namespace my_cartographer
{
  namespace map
  {

    struct CellLimits
    {
      CellLimits() = default;
      CellLimits(int init_num_x_cells, int init_num_y_cells)
          : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

      explicit CellLimits(const proto::CellLimits &cell_limits)
          : num_x_cells(cell_limits.num_x_cells()),
            num_y_cells(cell_limits.num_y_cells()) {}

      int num_x_cells = 0;
      int num_y_cells = 0;
    };

    inline proto::CellLimits ToProto(const CellLimits &cell_limits)
    {
      proto::CellLimits result;
      result.set_num_x_cells(cell_limits.num_x_cells);
      result.set_num_y_cells(cell_limits.num_y_cells);
      return result;
    }

    // Iterates in row-major order through a range of xy-indices.
    class XYIndexRangeIterator
        : public std::iterator<std::input_iterator_tag, Eigen::Array2i>
    {
    public:
      // Constructs a new iterator for the specified range.
      XYIndexRangeIterator(const Eigen::Array2i &min_xy_index,
                           const Eigen::Array2i &max_xy_index)
          : min_xy_index_(min_xy_index),
            max_xy_index_(max_xy_index),
            xy_index_(min_xy_index) {}

      // Constructs a new iterator for everything contained in 'cell_limits'.
      explicit XYIndexRangeIterator(const CellLimits &cell_limits)
          : XYIndexRangeIterator(Eigen::Array2i::Zero(),
                                 Eigen::Array2i(cell_limits.num_x_cells - 1,
                                                cell_limits.num_y_cells - 1)) {}

      XYIndexRangeIterator &operator++()
      {
        // This is a necessary evil. Bounds checking is very expensive and needs to
        // be avoided in production. We have unit tests that exercise this check
        // in debug mode.
        DCHECK(*this != end());
        if (xy_index_.x() < max_xy_index_.x())
        {
          ++xy_index_.x();
        }
        else
        {
          xy_index_.x() = min_xy_index_.x();
          ++xy_index_.y();
        }
        return *this;
      }

      Eigen::Array2i &operator*() { return xy_index_; }

      bool operator==(const XYIndexRangeIterator &other) const
      {
        return (xy_index_ == other.xy_index_).all();
      }

      bool operator!=(const XYIndexRangeIterator &other) const
      {
        return !operator==(other);
      }

      XYIndexRangeIterator begin()
      {
        return XYIndexRangeIterator(min_xy_index_, max_xy_index_);
      }

      XYIndexRangeIterator end()
      {
        XYIndexRangeIterator it = begin();
        it.xy_index_ = Eigen::Array2i(min_xy_index_.x(), max_xy_index_.y() + 1);
        return it;
      }

    private:
      Eigen::Array2i min_xy_index_;
      Eigen::Array2i max_xy_index_;
      Eigen::Array2i xy_index_;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_2D_XY_INDEX_H
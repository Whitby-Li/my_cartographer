//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_2D_GRID_2D_H
#define MY_CARTOGRAPHER_MAP_2D_GRID_2D_H

#include "my_cartographer/map/2d/map_limits.hpp"
#include "my_cartographer/map/grid_interface.h"
#include "my_cartographer/map/probability_values.h"
#include "my_cartographer/map/proto/grid_2d.pb.h"
#include "my_cartographer/map/proto/submap_visualization.pb.h"
#include "my_cartographer/map/proto/submaps_options_2d.pb.h"
#include "my_cartographer/map/value_conversion_tables.h"

namespace my_cartographer
{
  namespace map
  {

    proto::GridOptions2D CreateGridOptions2D(
        common::LuaParameterDictionary *const parameter_dictionary);

    enum class GridType
    {
      PROBABILITY_GRID,
      TSDF
    };

    class Grid2D : public GridInterface
    {
    public:
      Grid2D(const MapLimits &limits, float min_correspondence_cost,
             float max_correspondence_cost,
             ValueConversionTables *conversion_tables);
      explicit Grid2D(const proto::Grid2D &proto,
                      ValueConversionTables *conversion_tables);

      // Returns the limits of this Grid2D.
      const MapLimits &limits() const { return limits_; }

      // Finishes the update sequence.
      void FinishUpdate();

      // Returns the correspondence cost of the cell with 'cell_index'.
      float GetCorrespondenceCost(const Eigen::Array2i &cell_index) const
      {
        if (!limits().Contains(cell_index))
          return max_correspondence_cost_;
        return (*value_to_correspondence_cost_table_)
            [correspondence_cost_cells()[ToFlatIndex(cell_index)]];
      }

      virtual GridType GetGridType() const = 0;

      // Returns the minimum possible correspondence cost.
      float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

      // Returns the maximum possible correspondence cost.
      float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

      // Returns true if the probability at the specified index is known.
      bool IsKnown(const Eigen::Array2i &cell_index) const
      {
        return limits_.Contains(cell_index) &&
               correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
                   kUnknownCorrespondenceValue;
      }

      // Fills in 'offset' and 'limits' to define a subregion of that contains all
      // known cells.
      void ComputeCroppedLimits(Eigen::Array2i *const offset,
                                CellLimits *const limits) const;

      // Grows the map as necessary to include 'point'. This changes the meaning of
      // these coordinates going forward. This method must be called immediately
      // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
      virtual void GrowLimits(const Eigen::Vector2f &point);

      virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const = 0;

      virtual proto::Grid2D ToProto() const;

      virtual bool DrawToSubmapTexture(
          proto::SubmapQuery::Response::SubmapTexture *const texture,
          transform::Rigid3d local_pose) const = 0;

    protected:
      void GrowLimits(const Eigen::Vector2f &point,
                      const std::vector<std::vector<uint16> *> &grids,
                      const std::vector<uint16> &grids_unknown_cell_values);

      const std::vector<uint16> &correspondence_cost_cells() const
      {
        return correspondence_cost_cells_;
      }
      const std::vector<int> &update_indices() const { return update_indices_; }
      const Eigen::AlignedBox2i &known_cells_box() const
      {
        return known_cells_box_;
      }

      std::vector<uint16> *mutable_correspondence_cost_cells()
      {
        return &correspondence_cost_cells_;
      }

      std::vector<int> *mutable_update_indices() { return &update_indices_; }
      Eigen::AlignedBox2i *mutable_known_cells_box() { return &known_cells_box_; }

      // Converts a 'cell_index' into an index into 'cells_'.
      int ToFlatIndex(const Eigen::Array2i &cell_index) const
      {
        CHECK(limits_.Contains(cell_index)) << cell_index;
        return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
      }

    private:
      MapLimits limits_;
      std::vector<uint16> correspondence_cost_cells_;
      float min_correspondence_cost_;
      float max_correspondence_cost_;
      std::vector<int> update_indices_;

      // Bounding box of known cells to efficiently compute cropping limits.
      Eigen::AlignedBox2i known_cells_box_;
      const std::vector<float> *value_to_correspondence_cost_table_;
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_2D_GRID_2D_H
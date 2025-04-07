//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_2D_SUBMAP_2D_H
#define MY_CARTOGRAPHER_MAP_2D_SUBMAP_2D_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/map/2d/grid_2d.h"
#include "my_cartographer/map/2d/map_limits.hpp"
#include "my_cartographer/map/proto/serialization.pb.h"
#include "my_cartographer/map/proto/submap_visualization.pb.h"
#include "my_cartographer/map/proto/submaps_options_2d.pb.h"
#include "my_cartographer/map/range_data_inserter_interface.h"
#include "my_cartographer/map/submaps.h"
#include "my_cartographer/map/value_conversion_tables.h"
#include "my_cartographer/sensor/range_data.h"
#include "my_cartographer/transform/rigid_transform.h"

#include <Eigen/Core>

#include <memory>
#include <vector>

namespace my_cartographer
{
  namespace map
  {

    proto::SubmapsOptions2D CreateSubmapsOptions2D(
        common::LuaParameterDictionary *parameter_dictionary);

    class Submap2D : public Submap
    {
    public:
      Submap2D(const Eigen::Vector2f &origin, std::unique_ptr<Grid2D> grid,
               ValueConversionTables *conversion_tables);
      explicit Submap2D(const proto::Submap2D &proto,
                        ValueConversionTables *conversion_tables);

      proto::Submap ToProto(bool include_grid_data) const override;
      void UpdateFromProto(const proto::Submap &proto) override;

      void ToResponseProto(const transform::Rigid3d &global_submap_pose,
                           proto::SubmapQuery::Response *response) const override;

      const Grid2D *grid() const { return grid_.get(); }

      // Insert 'range_data' into this submap using 'range_data_inserter'. The
      // submap must not be finished yet.
      void InsertRangeData(const sensor::RangeData &range_data,
                           const RangeDataInserterInterface *range_data_inserter);
      void Finish();

    private:
      std::unique_ptr<Grid2D> grid_;
      ValueConversionTables *conversion_tables_;
    };

    // The first active submap will be created on the insertion of the first range
    // data. Except during this initialization when no or only one single submap
    // exists, there are always two submaps into which range data is inserted: an
    // old submap that is used for matching, and a new one, which will be used for
    // matching next, that is being initialized.
    //
    // Once a certain number of range data have been inserted, the new submap is
    // considered initialized: the old submap is no longer changed, the "new" submap
    // is now the "old" submap and is used for scan-to-map matching. Moreover, a
    // "new" submap gets created. The "old" submap is forgotten by this object.
    class ActiveSubmaps2D
    {
    public:
      explicit ActiveSubmaps2D(const proto::SubmapsOptions2D &options);

      ActiveSubmaps2D(const ActiveSubmaps2D &) = delete;
      ActiveSubmaps2D &operator=(const ActiveSubmaps2D &) = delete;

      // Inserts 'range_data' into the Submap collection.
      std::vector<std::shared_ptr<const Submap2D>> InsertRangeData(
          const sensor::RangeData &range_data);

      std::vector<std::shared_ptr<const Submap2D>> submaps() const;

    private:
      std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();
      std::unique_ptr<GridInterface> CreateGrid(const Eigen::Vector2f &origin);
      void FinishSubmap();
      void AddSubmap(const Eigen::Vector2f &origin);

      const proto::SubmapsOptions2D options_;
      std::vector<std::shared_ptr<Submap2D>> submaps_;
      std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;
      ValueConversionTables conversion_tables_;
    };
    
  }
}
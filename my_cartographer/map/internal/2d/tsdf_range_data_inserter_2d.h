//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_TSDF_RANGE_DATA_INSERTER_2D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_TSDF_RANGE_DATA_INSERTER_2D_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/map/internal/2d/tsdf_2d.h"
#include "my_cartographer/map/proto/tsdf_range_data_inserter_options_2d.pb.h"
#include "my_cartographer/map/range_data_inserter_interface.h"
#include "my_cartographer/sensor/point_cloud.h"
#include "my_cartographer/sensor/range_data.h"

namespace my_cartographer
{
  namespace map
  {

    proto::TSDFRangeDataInserterOptions2D CreateTSDFRangeDataInserterOptions2D(
        common::LuaParameterDictionary *parameter_dictionary);

    class TSDFRangeDataInserter2D : public RangeDataInserterInterface
    {
    public:
      explicit TSDFRangeDataInserter2D(
          const proto::TSDFRangeDataInserterOptions2D &options);

      TSDFRangeDataInserter2D(const TSDFRangeDataInserter2D &) = delete;
      TSDFRangeDataInserter2D &operator=(const TSDFRangeDataInserter2D &) = delete;

      // Casts a ray from origin towards hit for each hit in range data.
      // If 'options.update_free_space' is 'true', all cells along the ray
      // until 'truncation_distance' behind hit are updated. Otherwise, only the
      // cells within 'truncation_distance' around hit are updated.
      virtual void Insert(const sensor::RangeData &range_data,
                          GridInterface *grid) const override;

    private:
      void InsertHit(const proto::TSDFRangeDataInserterOptions2D &options,
                     const Eigen::Vector2f &hit, const Eigen::Vector2f &origin,
                     float normal, TSDF2D *tsdf) const;
      void UpdateCell(const Eigen::Array2i &cell, float update_sdf,
                      float update_weight, TSDF2D *tsdf) const;
      const proto::TSDFRangeDataInserterOptions2D options_;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_TSDF_RANGE_DATA_INSERTER_2D_H
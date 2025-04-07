//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_RANGE_DATA_INSERTER_INTERFACE_H
#define MY_CARTOGRAPHER_MAP_RANGE_DATA_INSERTER_INTERFACE_H

#include "my_cartographer/map/grid_interface.h"
#include "my_cartographer/map/proto/submaps_options_2d.pb.h"
#include "my_cartographer/sensor/range_data.h"

namespace my_cartographer
{
  namespace map
  {

    proto::RangeDataInserterOptions CreateRangeDataInserterOptions(
        common::LuaParameterDictionary *const parameter_dictionary);

    class RangeDataInserterInterface
    {
    public:
      virtual ~RangeDataInserterInterface() {}

      // Inserts 'range_data' into 'grid'.
      virtual void Insert(const sensor::RangeData &range_data,
                          GridInterface *grid) const = 0;
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_RANGE_DATA_INSERTER_INTERFACE_H
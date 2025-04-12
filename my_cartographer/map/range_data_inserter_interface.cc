//
// Created by whitby on 2025-04-05.
//

#include "my_cartographer/map/range_data_inserter_interface.h"
#include "my_cartographer/map/2d/probability_grid_range_data_inserter_2d.h"
#include "my_cartographer/map/internal/2d/tsdf_range_data_inserter_2d.h"

namespace my_cartographer
{
  namespace map
  {

    proto::RangeDataInserterOptions CreateRangeDataInserterOptions(
        common::LuaParameterDictionary *const parameter_dictionary)
    {
      proto::RangeDataInserterOptions options;
      const std::string range_data_inserter_type_string =
          parameter_dictionary->GetString("range_data_inserter_type");
      proto::RangeDataInserterOptions_RangeDataInserterType
          range_data_inserter_type;
      CHECK(proto::RangeDataInserterOptions_RangeDataInserterType_Parse(
          range_data_inserter_type_string, &range_data_inserter_type))
          << "Unknown RangeDataInserterOptions_RangeDataInserterType kind: "
          << range_data_inserter_type_string;
      options.set_range_data_inserter_type(range_data_inserter_type);
      *options.mutable_probability_grid_range_data_inserter_options_2d() =
          CreateProbabilityGridRangeDataInserterOptions2D(
              parameter_dictionary
                  ->GetDictionary("probability_grid_range_data_inserter")
                  .get());
      *options.mutable_tsdf_range_data_inserter_options_2d() =
          CreateTSDFRangeDataInserterOptions2D(
              parameter_dictionary->GetDictionary("tsdf_range_data_inserter")
                  .get());
      return options;
    }
  } // namespace map
} // namespace my_cartographer
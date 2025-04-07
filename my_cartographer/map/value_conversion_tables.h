//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_VALUE_CONVERSION_TABLES_H
#define MY_CARTOGRAPHER_MAP_VALUE_CONVERSION_TABLES_H

#include "my_cartographer/common/port.hpp"

#include <map>
#include <vector>
#include <glog/logging.h>

namespace my_cartographer
{
  namespace map
  {

    // Performs lazy computations of looku tables for mapping from a unit16 value to a float in ['lower_bound', 'upper_bound'].
    // The first element of the table is set to 'unknown_result'.
    class ValueConversionTables
    {
    public:
      const std::vector<float> *GetConversionTable(float unknown_result,
                                                   float lower_bound,
                                                   float upper_bound);

    private:
      std::map<const std::tuple<float /* unknown_result */, float /* lower_bound */,
                                float /* upper_bound */>,
               std::unique_ptr<const std::vector<float>>>
          bounds_to_lookup_table_;
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_VALUE_CONVERSION_TABLES_H
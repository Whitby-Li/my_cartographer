//
// Created by whitby on 2025-04-12.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/map/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.h"

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      proto::RealTimeCorrelativeScanMatcherOptions CreateRealTimeCorrelativeScanMatcherOptions(
          common::LuaParameterDictionary *const parameter_dictionary);

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H
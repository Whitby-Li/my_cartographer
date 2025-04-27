//
// Created by whitby on 2025-04-12.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_OPTIONS_3D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_OPTIONS_3D_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/map/proto/local_trajectory_builder_options_3d.pb.h"

namespace my_cartographer
{
  namespace map
  {

    proto::LocalTrajectoryBuilderOptions3D CreateLocalTrajectoryBuilderOptions3D(
        common::LuaParameterDictionary *parameter_dictionary);

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_OPTIONS_3D_H
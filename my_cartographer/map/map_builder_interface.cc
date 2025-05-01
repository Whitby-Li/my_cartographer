//
// Created by whitby on 2025-04-26.
//

#include "my_cartographer/map/map_builder_interface.h"
#include "my_cartographer/map/pose_graph.h"

namespace my_cartographer
{
  namespace map
  {

    proto::MapBuilderOptions CreateMapBuilderOptions(
        common::LuaParameterDictionary *const parameter_dictionary)
    {
      proto::MapBuilderOptions options;
      options.set_use_trajectory_builder_2d(
          parameter_dictionary->GetBool("use_trajectory_builder_2d"));
      options.set_use_trajectory_builder_3d(
          parameter_dictionary->GetBool("use_trajectory_builder_3d"));
      options.set_num_background_threads(
          parameter_dictionary->GetNonNegativeInt("num_background_threads"));
      options.set_collate_by_trajectory(
          parameter_dictionary->GetBool("collate_by_trajectory"));
      *options.mutable_pose_graph_options() = CreatePoseGraphOptions(
          parameter_dictionary->GetDictionary("pose_graph").get());
      CHECK_NE(options.use_trajectory_builder_2d(),
               options.use_trajectory_builder_3d());
      return options;
    }

  }  // namespace map
}  // namespace my_cartographer
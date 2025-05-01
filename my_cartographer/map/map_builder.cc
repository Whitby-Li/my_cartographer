//
// Created by whitby on 2025-04-26.
//

#include "my_cartographer/map/map_builder.h"

#include <absl/memory/memory.h>
#include <absl/types/optional.h>

#include "my_cartographer/common/time.h"
#include "my_cartographer/io/internal/mapping_state_serialization.h"
#include "my_cartographer/io/proto_stream.h"
#include "my_cartographer/io/proto_stream_deserializer.h"
#include "my_cartographer/io/serialization_format_migration.h"
#include "my_cartographer/map/internal/2d/local_trajectory_builder_2d.h"
#include "my_cartographer/map/internal/2d/pose_graph_2d.h"
#include "my_cartographer/map/internal/3d/local_trajectory_builder_3d.h"
#include "my_cartographer/map/internal/3d/pose_graph_3d.h"
#include "my_cartographer/map/internal/collated_trajectory_builder.h"
#include "my_cartographer/map/internal/global_trajectory_builder.h"
#include "my_cartographer/map/internal/motion_filter.h"
#include "my_cartographer/sensor/internal/collator.h"
#include "my_cartographer/sensor/internal/trajectory_collator.h"
#include "my_cartographer/sensor/internal/voxel_filter.h"
#include "my_cartographer/transform/transform.h"
//
// Created by whitby on 2025-03-29.
//

#include "my_cartographer/map/trajectory_builder_interface.h"
#include "my_cartographer/map/internal/2d/local_trajectory_builder_2d.h"
#include "my_cartographer/map/internal/3d/local_trajectory_builder_3d.h"
#include "my_cartographer/map/internal/local_slam_result_data.h"

namespace my_cartographer
{
  namespace map
  {

    namespace
    {

      void PopulatePureLocalizationTrimmerOptions(
          proto::TrajectoryBuilderOptions *const trajectory_builder_options,
          common::LuaParameterDictionary *const parameter_dictionary)
      {
        constexpr char kDictionaryKey[] = "pure_localization_trimmer";
        if (!parameter_dictionary->HasKey(kDictionaryKey))
          return;

        auto options_dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
        auto *options =
            trajectory_builder_options->mutable_pure_localization_trimmer();
        options->set_max_submaps_to_keep(
            options_dictionary->GetInt("max_submaps_to_keep"));
      }

      void PopulatePoseGraphOdometryMotionFilterOptions(
          proto::TrajectoryBuilderOptions *const trajectory_builder_options,
          common::LuaParameterDictionary *const parameter_dictionary)
      {
        constexpr char kDictionaryKey[] = "pose_graph_odometry_motion_filter";
        if (!parameter_dictionary->HasKey(kDictionaryKey))
          return;

        auto options_dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
        auto *options =
            trajectory_builder_options->mutable_pose_graph_odometry_motion_filter();
        options->set_max_time_seconds(
            options_dictionary->GetDouble("max_time_seconds"));
        options->set_max_distance_meters(
            options_dictionary->GetDouble("max_distance_meters"));
        options->set_max_angle_radians(
            options_dictionary->GetDouble("max_angle_radians"));
      }

    } // namespace

    proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
        common::LuaParameterDictionary *const parameter_dictionary)
    {
      proto::TrajectoryBuilderOptions options;
      *options.mutable_trajectory_builder_2d_options() =
          CreateLocalTrajectoryBuilderOptions2D(
              parameter_dictionary->GetDictionary("trajectory_builder_2d").get());
      *options.mutable_trajectory_builder_3d_options() =
          CreateLocalTrajectoryBuilderOptions3D(
              parameter_dictionary->GetDictionary("trajectory_builder_3d").get());
      options.set_collate_fixed_frame(
          parameter_dictionary->GetBool("collate_fixed_frame"));
      options.set_collate_landmarks(
          parameter_dictionary->GetBool("collate_landmarks"));
      PopulatePureLocalizationTrimmerOptions(&options, parameter_dictionary);
      PopulatePoseGraphOdometryMotionFilterOptions(&options, parameter_dictionary);
      return options;
    }

    proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId &sensor_id)
    {
      proto::SensorId sensor_id_proto;
      switch (sensor_id.type)
      {
      case TrajectoryBuilderInterface::SensorId::SensorType::RANGE:
        sensor_id_proto.set_type(proto::SensorId::RANGE);
        break;
      case TrajectoryBuilderInterface::SensorId::SensorType::IMU:
        sensor_id_proto.set_type(proto::SensorId::IMU);
        break;
      case TrajectoryBuilderInterface::SensorId::SensorType::ODOMETRY:
        sensor_id_proto.set_type(proto::SensorId::ODOMETRY);
        break;
      case TrajectoryBuilderInterface::SensorId::SensorType::FIXED_FRAME_POSE:
        sensor_id_proto.set_type(proto::SensorId::FIXED_FRAME_POSE);
        break;
      case TrajectoryBuilderInterface::SensorId::SensorType::LANDMARK:
        sensor_id_proto.set_type(proto::SensorId::LANDMARK);
        break;
      case TrajectoryBuilderInterface::SensorId::SensorType::LOCAL_SLAM_RESULT:
        sensor_id_proto.set_type(proto::SensorId::LOCAL_SLAM_RESULT);
        break;
      default:
        LOG(FATAL) << "Unsupported sensor type.";
      }
      sensor_id_proto.set_id(sensor_id.id);
      return sensor_id_proto;
    }

    TrajectoryBuilderInterface::SensorId FromProto(const proto::SensorId &sensor_id_proto)
    {
      TrajectoryBuilderInterface::SensorId sensor_id;
      switch (sensor_id_proto.type())
      {
      case proto::SensorId::RANGE:
        sensor_id.type = TrajectoryBuilderInterface::SensorId::SensorType::RANGE;
        break;
      case proto::SensorId::IMU:
        sensor_id.type = TrajectoryBuilderInterface::SensorId::SensorType::IMU;
        break;
      case proto::SensorId::ODOMETRY:
        sensor_id.type = TrajectoryBuilderInterface::SensorId::SensorType::ODOMETRY;
        break;
      case proto::SensorId::FIXED_FRAME_POSE:
        sensor_id.type = TrajectoryBuilderInterface::SensorId::SensorType::FIXED_FRAME_POSE;
        break;
      case proto::SensorId::LANDMARK:
        sensor_id.type = TrajectoryBuilderInterface::SensorId::SensorType::LANDMARK;
        break;
      case proto::SensorId::LOCAL_SLAM_RESULT:
        sensor_id.type = TrajectoryBuilderInterface::SensorId::SensorType::LOCAL_SLAM_RESULT;
        break;
      default:
        LOG(FATAL) << "Unsupported sensor type.";
      }
      sensor_id.id = sensor_id_proto.id();
      return sensor_id;
    }

  } // namespace map
} // namespace my_cartographer
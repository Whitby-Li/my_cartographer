//
// Created by whitby on 2025-03-29.
//

#ifndef MY_CARTOGRAPHER_MAP_TRAJECTORY_BUILDER_INTERFACE_H
#define MY_CARTOGRAPHER_MAP_TRAJECTORY_BUILDER_INTERFACE_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/common/port.hpp"
#include "my_cartographer/common/time.h"
#include "my_cartographer/sensor/fixed_frame_pose_data.h"
#include "my_cartographer/sensor/imu_data.h"
#include "my_cartographer/sensor/landmark_data.h"
#include "my_cartographer/sensor/odometry_data.h"
#include "my_cartographer/sensor/timed_point_cloud_data.h"
#include "my_cartographer/map/proto/trajectory_builder_options.pb.h"
#include "my_cartographer/map/submaps.h"

#include <string>
#include <memory>
#include <functional>

namespace my_cartographer
{
  namespace map
  {

    proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
        common::LuaParameterDictionary *const parameter_dictionary);

    class LocalSlamResultData;

    /**
     * @brief Interface used for both 2D and 3D SLAM
     * @note wire up a global SLAM stack, i.e. local SLAM for initial pose estimates,
     * scan matching to detect loop closures, and a sparse graph optimization to compute optimized pose estimates.
     */
    class TrajectoryBuilderInterface
    {
    public:
      struct InsertionResult
      {
        NodeId node_id;
        std::shared_ptr<const TrajectoryNode::Data> constant_data;
        std::vector<std::shared_ptr<const Submap>> insertion_submaps;
      };

      /**
       * @brief A callback which is called after local SLAM processes an accumulated `sensor::RangeData`.
       */
      using LocalSlamResultCallback = std::function<void(int, common::Time, transform::Rigid3d,
                                                         sensor::RangeData, std::unique_ptr<const InsertionResult>)>;

      struct SensorId
      {
        enum class SensorType
        {
          RANGE = 0,
          IMU,
          ODOMETRY,
          FIXED_FRAME_POSE,
          LANDMARK,
          LOCAL_SLAM_RESULT
        };

        SensorType type;
        std::string id;

        bool operator==(const SensorId &other) const
        {
          return std::forward_as_tuple(type, id) ==
                 std::forward_as_tuple(other.type, other.id);
        }

        bool operator<(const SensorId &other) const
        {
          return std::forward_as_tuple(type, id) <
                 std::forward_as_tuple(other.type, other.id);
        }
      };

      TrajectoryBuilderInterface() {}
      virtual ~TrajectoryBuilderInterface() {}

      TrajectoryBuilderInterface(const TrajectoryBuilderInterface &) = delete;
      TrajectoryBuilderInterface &operator=(const TrajectoryBuilderInterface &) =
          delete;

      virtual void AddSensorData(
          const std::string &sensor_id,
          const sensor::TimedPointCloudData &timed_point_cloud_data) = 0;
      virtual void AddSensorData(const std::string &sensor_id,
                                 const sensor::ImuData &imu_data) = 0;
      virtual void AddSensorData(const std::string &sensor_id,
                                 const sensor::OdometryData &odometry_data) = 0;
      virtual void AddSensorData(
          const std::string &sensor_id,
          const sensor::FixedFramePoseData &fixed_frame_pose) = 0;
      virtual void AddSensorData(const std::string &sensor_id, const sensor::LandmarkData &landmark_data) = 0;
      // Allows to directly add local SLAM results to the 'PoseGraph'. Note that it
      // is invalid to add local SLAM results for a trajectory that has a
      // 'LocalTrajectoryBuilder2D/3D'.
      virtual void AddLocalSlamResultData( std::unique_ptr<LocalSlamResultData> local_slam_result_data) = 0;
    };

    proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId &sensor_id);
    TrajectoryBuilderInterface::SensorId FromProto(const proto::SensorId &sensor_id_proto);

  } // namespace map
} // namespace my_cartographer


#endif // MY_CARTOGRAPHER_MAP_TRAJECTORY_BUILDER_INTERFACE_H
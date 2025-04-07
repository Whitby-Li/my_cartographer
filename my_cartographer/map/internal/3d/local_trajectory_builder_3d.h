//
// Created by whitby on 2025-04-07.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H

#include "my_cartographer/common/time.h"
#include "my_cartographer/map/3d/submap_3d.h"
#include "my_cartographer/map/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "my_cartographer/map/internal/3d/scan_matching/real_time_correlative_scan_matcher_3d.h"
#include "my_cartographer/map/internal/motion_filter.h"
#include "my_cartographer/map/internal/range_data_collator.h"
#include "my_cartographer/map/pose_extrapolator_interface.h"
#include "my_cartographer/map/proto/local_trajectory_builder_options_3d.pb.h"
#include "my_cartographer/metrics/family_factory.hpp"
#include "my_cartographer/sensor/imu_data.h"
#include "my_cartographer/sensor/internal/voxel_filter.h"
#include "my_cartographer/sensor/odometry_data.h"
#include "my_cartographer/sensor/range_data.h"
#include "my_cartographer/sensor/timed_point_cloud_data.h"
#include "my_cartographer/transform/rigid_transform.h"

#include <memory>
#include <chrono>

namespace my_cartographer
{
  namespace map
  {

    // Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
    // without loop closure.
    class LocalTrajectoryBuilder3D
    {
    public:
      struct InsertionResult
      {
        std::shared_ptr<const TrajectoryNode::Data> constant_data;
        std::vector<std::shared_ptr<const Submap3D>> insertion_submaps;
      };
      struct MatchingResult
      {
        common::Time time;
        transform::Rigid3d local_pose;
        sensor::RangeData range_data_in_local;
        // 'nullptr' if dropped by the motion filter.
        std::unique_ptr<const InsertionResult> insertion_result;
      };

      explicit LocalTrajectoryBuilder3D(
          const proto::LocalTrajectoryBuilderOptions3D &options,
          const std::vector<std::string> &expected_range_sensor_ids);
      ~LocalTrajectoryBuilder3D();

      LocalTrajectoryBuilder3D(const LocalTrajectoryBuilder3D &) = delete;
      LocalTrajectoryBuilder3D &operator=(const LocalTrajectoryBuilder3D &) = delete;

      void AddImuData(const sensor::ImuData &imu_data);
      // Returns 'MatchingResult' when range data accumulation completed,
      // otherwise 'nullptr'.  `TimedPointCloudData::time` is when the last point in
      // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
      // relative time of point with respect to `TimedPointCloudData::time`.
      std::unique_ptr<MatchingResult> AddRangeData(
          const std::string &sensor_id,
          const sensor::TimedPointCloudData &range_data);
      void AddOdometryData(const sensor::OdometryData &odometry_data);

      static void RegisterMetrics(metrics::FamilyFactory *family_factory);

    private:
      std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
          common::Time time,
          const sensor::RangeData &filtered_range_data_in_tracking,
          const absl::optional<common::Duration> &sensor_duration,
          const transform::Rigid3d &pose_prediction,
          const Eigen::Quaterniond &gravity_alignment);

      std::unique_ptr<InsertionResult> InsertIntoSubmap(
          common::Time time, const sensor::RangeData &filtered_range_data_in_local,
          const sensor::RangeData &filtered_range_data_in_tracking,
          const sensor::PointCloud &high_resolution_point_cloud_in_tracking,
          const sensor::PointCloud &low_resolution_point_cloud_in_tracking,
          const transform::Rigid3d &pose_estimate,
          const Eigen::Quaterniond &gravity_alignment);

      // Scan matches using the two point clouds and returns the observed pose, or
      // nullptr on failure.
      std::unique_ptr<transform::Rigid3d> ScanMatch(
          const transform::Rigid3d &pose_prediction,
          const sensor::PointCloud &low_resolution_point_cloud_in_tracking,
          const sensor::PointCloud &high_resolution_point_cloud_in_tracking);

      const proto::LocalTrajectoryBuilderOptions3D options_;
      ActiveSubmaps3D active_submaps_;

      MotionFilter motion_filter_;
      std::unique_ptr<scan_matching::RealTimeCorrelativeScanMatcher3D>
          real_time_correlative_scan_matcher_;
      std::unique_ptr<scan_matching::CeresScanMatcher3D> ceres_scan_matcher_;

      std::unique_ptr<PoseExtrapolatorInterface> extrapolator_;

      int num_accumulated_ = 0;
      std::vector<sensor::TimedPointCloudOriginData>
          accumulated_point_cloud_origin_data_;
      absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;

      absl::optional<double> last_thread_cpu_time_seconds_;

      RangeDataCollator range_data_collator_;

      absl::optional<common::Time> last_sensor_time_;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_3D_LOCAL_TRAJECTORY_BUILDER_3D_H
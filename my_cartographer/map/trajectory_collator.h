//
// Created by whitby on 2025-04-04.
//

#ifndef MY_CARTOGRAPHER_MAP_TRAJECTORY_COLLATOR_H
#define MY_CARTOGRAPHER_MAP_TRAJECTORY_COLLATOR_H

#include "my_cartographer/metrics/counter.h"
#include "my_cartographer/metrics/family_factory.hpp"
#include "my_cartographer/map/collator_interface.h"
#include "my_cartographer/map/ordered_multi_queue.h"

#include <absl/container/flat_hash_map.h>

namespace my_cartographer
{
  namespace map
  {

    // Waits to see at least one data item for all sensor ids and dispatches data
    // in merge-sorted order. Contrary to 'Collator', it does not wait for other
    // trajectories.
    // Also contrary to 'Collator', whose output is deterministic, the sequence in
    // which data is dispatched is not sorted, so non-deterministic input sequences
    // will result in non-deterministic output.
    class TrajectoryCollator : public CollatorInterface
    {
    public:
      TrajectoryCollator() {}

      TrajectoryCollator(const TrajectoryCollator &) = delete;
      TrajectoryCollator &operator=(const TrajectoryCollator &) = delete;

      void AddTrajectory(
          int trajectory_id,
          const absl::flat_hash_set<std::string> &expected_sensor_ids,
          const Callback &callback) override;

      void FinishTrajectory(int trajectory_id) override;

      void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;

      void Flush() override;

      absl::optional<int> GetBlockingTrajectoryId() const override;

      static void RegisterMetrics(metrics::FamilyFactory *family_factory);

    private:
      metrics::Counter *GetOrCreateSensorMetric(const std::string &sensor_id,
                                                int trajectory_id);

      static metrics::Family<metrics::Counter> *
          collator_metrics_family_;

      // Holds individual counters for each trajectory/sensor pair.
      absl::flat_hash_map<std::string, metrics::Counter *> metrics_map_;

      absl::flat_hash_map<int, OrderedMultiQueue> trajectory_to_queue_;

      // Map of trajectory ID to all associated QueueKeys.
      absl::flat_hash_map<int, std::vector<QueueKey>> trajectory_to_queue_keys_;
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_TRAJECTORY_COLLATOR_H
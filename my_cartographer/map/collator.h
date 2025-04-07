//
// Created by whitby on 2025-04-04.
//

#ifndef MY_CARTOGRAPHER_MAP_COLLATOR_H
#define MY_CARTOGRAPHER_MAP_COLLATOR_H

#include "my_cartographer/map/collator_interface.h"
#include "my_cartographer/map/data.h"
#include "my_cartographer/map/ordered_multi_queue.h"

#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>

#include <functional>
#include <memory>
#include <vector>

namespace my_cartographer
{
  namespace map
  {

    class Collator : public CollatorInterface
    {
    public:
      Collator() {}

      Collator(const Collator &) = delete;
      Collator &operator=(const Collator &) = delete;

      void AddTrajectory(
          int trajectory_id,
          const absl::flat_hash_set<std::string> &expected_sensor_ids,
          const Callback &callback) override;

      void FinishTrajectory(int trajectory_id) override;

      void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;

      void Flush() override;

      absl::optional<int> GetBlockingTrajectoryId() const override;

    private:
      // Queue keys are a pair of trajectory ID and sensor identifier.
      OrderedMultiQueue queue_;

      // Map of trajectory ID to all associated QueueKeys.
      absl::flat_hash_map<int, std::vector<QueueKey>> queue_keys_;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_COLLATOR_H
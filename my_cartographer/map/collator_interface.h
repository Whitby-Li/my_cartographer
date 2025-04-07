//
// Created by whitby on 2025-04-04.
//

#ifndef MY_CARTOGRAPHER_MAP_COLLATOR_INTERFACE_H
#define MY_CARTOGRAPHER_MAP_COLLATOR_INTERFACE_H

#include "my_cartographer/map/data.h"

#include <absl/container/flat_hash_set.h>
#include <absl/types/optional.h>

#include <memory>
#include <functional>
#include <vector>

namespace my_cartographer
{
  namespace map
  {
    class CollatorInterface
    {
    public:
      using Callback =
          std::function<void(const std::string &, std::unique_ptr<Data>)>;

      CollatorInterface() {}
      virtual ~CollatorInterface() {}
      CollatorInterface(const CollatorInterface &) = delete;
      CollatorInterface &operator=(const CollatorInterface &) = delete;

      // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
      // for each collated sensor data.
      virtual void AddTrajectory(
          int trajectory_id,
          const absl::flat_hash_set<std::string> &expected_sensor_ids,
          const Callback &callback) = 0;

      // Marks 'trajectory_id' as finished.
      virtual void FinishTrajectory(int trajectory_id) = 0;

      // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
      // sensor data. Sensor packets with matching 'data.sensor_id_' must be added
      // in time order.
      virtual void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) = 0;

      // Dispatches all queued sensor packets. May only be called once.
      // AddSensorData may not be called after Flush.
      virtual void Flush() = 0;

      // Must only be called if at least one unfinished trajectory exists. Returns
      // the ID of the trajectory that needs more data before CollatorInterface is
      // unblocked. Returns 'nullopt' for implementations that do not wait for a
      // particular trajectory.
      virtual absl::optional<int> GetBlockingTrajectoryId() const = 0;
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_COLLATOR_INTERFACE_H
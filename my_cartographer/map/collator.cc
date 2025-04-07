//
// Created by whitby on 2025-04-04.
//

#include "my_cartographer/map/collator.h"

namespace my_cartographer
{
  namespace map
  {

    void Collator::AddTrajectory(
        const int trajectory_id,
        const absl::flat_hash_set<std::string> &expected_sensor_ids,
        const Callback &callback)
    {
      for (const auto &sensor_id : expected_sensor_ids)
      {
        const auto queue_key = QueueKey{trajectory_id, sensor_id};
        queue_.AddQueue(queue_key,
                        [callback, sensor_id](std::unique_ptr<Data> data) {
                          callback(sensor_id, std::move(data));
                        });
        queue_keys_[trajectory_id].push_back(queue_key);
      }
    }

    void Collator::FinishTrajectory(const int trajectory_id)
    {
      for (const auto &queue_key : queue_keys_[trajectory_id])
      {
        queue_.MarkQueueAsFinished(queue_key);
      }
    }

    void Collator::AddSensorData(const int trajectory_id,
                                 std::unique_ptr<Data> data)
    {
      QueueKey queue_key{trajectory_id, data->GetSensorId()};
      queue_.Add(std::move(queue_key), std::move(data));
    }

    void Collator::Flush() { queue_.Flush(); }

    absl::optional<int> Collator::GetBlockingTrajectoryId() const
    {
      return absl::optional<int>(queue_.GetBlocker().trajectory_id);
    }

  } // namespace map
} // namespace my_cartographer
//
// Created by whitby on 2025-03-29.
//

#ifndef MY_CARTOGRAPHER_MAP_ORDERED_MULTI_QUEUE_H
#define MY_CARTOGRAPHER_MAP_ORDERED_MULTI_QUEUE_H

#include "my_cartographer/common/internal/blocking_queue.hpp"
#include "my_cartographer/common/time.h"
#include "my_cartographer/common/port.hpp"
#include "my_cartographer/map/dispatchable.hpp"

namespace my_cartographer
{
  namespace map
  {

    struct QueueKey
    {
      int trajectory_id;
      std::string sensor_id;

      bool operator<(const QueueKey &other) const
      {
        return std::forward_as_tuple(trajectory_id, sensor_id) <
               std::forward_as_tuple(other.trajectory_id, other.sensor_id);
      }
    };

    /**
     * @brief Maintains multiple queues of sorted sensor data and dispatches it in merge sorted order.
     * @note It will wait to see at lease on e value for each unfinished queue before dispatching the next time ordered value across all queues.
     * This class is thread-compatible.
     */
    class OrderedMultiQueue
    {
    public:
      using Callback = std::function<void(std::unique_ptr<Data>)>;

      OrderedMultiQueue();
      OrderedMultiQueue(OrderedMultiQueue &&queue) = default;

      ~OrderedMultiQueue();

      // Adds a new queue with key 'queue_key' which must not already exist.
      // 'callback' will be called whenever data from this queue can be dispatched.
      void AddQueue(const QueueKey &queue_key, Callback callback);

      // Marks a queue as finished, i.e. no further data can be added. The queue
      // will be removed once the last piece of data from it has been dispatched.
      void MarkQueueAsFinished(const QueueKey &queue_key);

      // Adds 'data' to a queue with the given 'queue_key'. Data must be added
      // sorted per queue.
      void Add(const QueueKey &queue_key, std::unique_ptr<Data> data);

      // Dispatches all remaining values in sorted order and removes the underlying
      // queues.
      void Flush();

      // Must only be called if at least one unfinished queue exists. Returns the
      // key of a queue that needs more data before the OrderedMultiQueue can
      // dispatch data.
      QueueKey GetBlocker() const;

    private:
      struct Queue
      {
        common::BlockingQueue<std::unique_ptr<Data>> queue;
        Callback callback;
        bool finished = false;
      };

      void Dispatch();
      void CannotMakeProgress(const QueueKey &queue_key);
      common::Time GetCommonStartTime(int trajectory_id);

      // Used to verify that values are dispatched in sorted order.
      common::Time last_dispatched_time_ = common::Time::min();

      std::map<int, common::Time> common_start_time_per_trajectory_;
      std::map<QueueKey, Queue> queues_;
      QueueKey blocker_;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_ORDERED_MULTI_QUEUE_H
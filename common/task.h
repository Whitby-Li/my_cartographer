//
// Created by whitby on 2025-02-03.
//

#ifndef MY_CARTOGRAPHER_COMMON_TASK_H_
#define MY_CARTOGRAPHER_COMMON_TASK_H_

#include "thread_pool.h"

#include <absl/synchronization/mutex.h>
#include <glog/logging.h>

#include <set>
#include <memory>
#include <functional>

namespace cartographer
{
  namespace common
  {
    class ThreadPoolInterface;

    class Task
    {
    public:
      friend class ThreadPoolInterface;

      using WorkItem = std::function<void()>;
      enum State
      {
        NEW,
        DISPATCHED,
        DEPENDENCIES_COMPLETED,
        RUNNING,
        COMPLETED
      };

      Task() = default;
      ~Task();

      State GetState() LOCKS_EXCLUDED(mutex_);

      // State must be 'NEW'.
      void SetWorkItem(const WorkItem &work_item) LOCKS_EXCLUDED(mutex_);

      // State must be 'NEW'. 'dependency' may be nullptr, in which case it is
      // assumed completed.
      void AddDependency(std::weak_ptr<Task> dependency) LOCKS_EXCLUDED(mutex_);

    private:
      // Allowed in all states.
      void AddDependentTask(Task *dependent_task);

      // State must be 'DEPENDENCIES_COMPLETED' and becomes 'COMPLETED'.
      void Execute() LOCKS_EXCLUDED(mutex_);

      // State must be 'NEW' and becomes 'DISPATCHED' or 'DEPENDENCIES_COMPLETED'.
      void SetThreadPool(ThreadPoolInterface *thread_pool) LOCKS_EXCLUDED(mutex_);

      // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become
      // 'DEPENDENCIES_COMPLETED'.
      void OnDependenyCompleted();

      WorkItem work_item_ GUARDED_BY(mutex_);
      ThreadPoolInterface *thread_pool_to_notify_ GUARDED_BY(mutex_) = nullptr;
      State state_ GUARDED_BY(mutex_) = NEW;
      unsigned int uncompleted_dependencies_ GUARDED_BY(mutex_) = 0;
      std::set<Task *> dependent_tasks_ GUARDED_BY(mutex_);

      absl::Mutex mutex_;
    };

  } // namespace common
} // namespace cartographer

#endif // MY_CARTOGRAPHER_COMMON_TASK_H_

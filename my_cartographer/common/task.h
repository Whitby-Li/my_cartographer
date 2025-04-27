//
// Created by whitby on 2025-02-03.
//

#ifndef MY_CARTOGRAPHER_COMMON_TASK_H_
#define MY_CARTOGRAPHER_COMMON_TASK_H_

#include "my_cartographer/common/thread_pool.h"

#include <absl/synchronization/mutex.h>
#include <glog/logging.h>

#include <set>
#include <memory>
#include <functional>

namespace my_cartographer
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

      /**
       * @brief 设置执行任务内容
       * @param work_item 任务内容函数指针
       * @note State must be 'NEW'.
       */
      void SetWorkItem(const WorkItem &work_item) LOCKS_EXCLUDED(mutex_);

      /**
       * @brief 添加当前任务的依赖任务
       * @param dependency 依赖任务，可为 nullptr(如果已完成)
       * @note State must be 'NEW'.
       */
      void AddDependency(std::weak_ptr<Task> dependency) LOCKS_EXCLUDED(mutex_);

    private:
      // Allowed in all states.
      /**
       * @brief 添加被依赖任务，但当前任务完成后会通知依赖任务
       */
      void AddDependentTask(Task *dependent_task);

      /**
       * @brief 执行任务
       * @note State must be 'DEPENDENCIES_COMPLETED' and becomes 'COMPLETED'.
       */
      void Execute() LOCKS_EXCLUDED(mutex_);

      /**
       * @brief 设置线程池
       * @param thread_pool 线程池
       * @note State must be 'NEW' and becomes 'DISPATCHED' or 'DEPENDENCIES_COMPLETED'.
       */
      void SetThreadPool(ThreadPoolInterface *thread_pool) LOCKS_EXCLUDED(mutex_);

      // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become
      // 'DEPENDENCIES_COMPLETED'.
      /**
       * @brief 依赖任务完成后，通知当前任务；若当前任务的依赖任务都完成，则通知线程池，状态变更为 'DEPENDENCIES_COMPLETED'
       */
      void OnDependenyCompleted();

      WorkItem work_item_ GUARDED_BY(mutex_);
      ThreadPoolInterface *thread_pool_to_notify_ GUARDED_BY(mutex_) = nullptr;
      State state_ GUARDED_BY(mutex_) = NEW;
      unsigned int uncompleted_dependencies_ GUARDED_BY(mutex_) = 0;
      std::set<Task *> dependent_tasks_ GUARDED_BY(mutex_);

      absl::Mutex mutex_;
    };

  } // namespace common
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_COMMON_TASK_H_

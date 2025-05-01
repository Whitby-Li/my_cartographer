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

      using WorkItem = std::function<void()>; // 任务内容，具体类型为 std::function<void()>，可任意调用无参数、无返回值的对象

      // Task 运行状态
      enum State
      {
        NEW,                      // 新创建，还未调度
        DISPATCHED,               // 已被调度，等待依赖 Task 完成
        DEPENDENCIES_COMPLETED,   // 所有依赖 Task 已完成
        RUNNING,                  // 正在执行中
        COMPLETED                 // 执行完成
      };

      Task() = default;
      ~Task();

      /**
       * @brief 获取 Task 状态
       */
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
      /**
       * @brief 添加被依赖任务，但当前任务完成后会通知依赖任务
       */
      void AddDependentTask(Task *dependent_task);

      /**
       * @brief 执行任务
       * @note 前置状态 DEPENDENCIES_COMPLETED；后置状态 COMPLETED
       */
      void Execute() LOCKS_EXCLUDED(mutex_);

      /**
       * @brief 设置线程池
       * @param thread_pool 线程池
       * @note 任务必须处于还未放入线程池状态（如 NEW, DISPATCHED, DEPENDENCIES_COMPLETED）
       */
      void SetThreadPool(ThreadPoolInterface *thread_pool) LOCKS_EXCLUDED(mutex_);

      /**
       * @brief 通知当前任务，一个依赖项完成；若当前任务的依赖任务都完成，则通知线程池，且状态变更为 'DEPENDENCIES_COMPLETED'
       */
      void OnDependenyCompleted();

      WorkItem work_item_ GUARDED_BY(mutex_);     // 任务执行内容
      ThreadPoolInterface *thread_pool_to_notify_ GUARDED_BY(mutex_) = nullptr; // 线程池指针
      State state_ GUARDED_BY(mutex_) = NEW;      // 任务状态
      unsigned int uncompleted_dependencies_ GUARDED_BY(mutex_) = 0;  // 依赖任务未完成数量统计
      std::set<Task *> dependent_tasks_ GUARDED_BY(mutex_);           // 被依赖任务集合，当该任务运行完成后，通知它们

      absl::Mutex mutex_;
    };

  } // namespace common
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_COMMON_TASK_H_

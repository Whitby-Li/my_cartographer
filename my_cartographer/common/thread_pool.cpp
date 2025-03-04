//
// Created by whitby on 2025-02-04.
//

#include "my_cartographer/common/thread_pool.h"
#include "my_cartographer/common/task.h"

#include <absl/memory/memory.h>
#include <glog/logging.h>

#ifdef WIN32
#include <unistd.h>
#endif
#include <algorithm>
#include <chrono>
#include <numeric>

namespace my_cartographer
{
  namespace common
  {
    void ThreadPoolInterface::Execute(Task *task)
    {
      task->Execute();
    }

    void ThreadPoolInterface::SetThreadPool(Task *task)
    {
      task->SetThreadPool(this);
    }

    ThreadPool::ThreadPool(int num_threads)
    {
      CHECK_GT(num_threads, 0) << "ThreadPool requires a positive num_threads!";
      absl::MutexLock locker(&mutex_);
      for (int i = 0; i != num_threads; ++i)
      {
        pool_.emplace_back([this]()
                           { ThreadPool::DoWork(); });
      }
    }

    ThreadPool::~ThreadPool()
    {
      {
        absl::MutexLock locker(&mutex_);
        CHECK(running_);
        running_ = false;
      }

      for (std::thread &thread : pool_)
      {
        thread.join();
      }
    }

    void ThreadPool::NotifyDependenciesCompleted(Task *task)
    {
      absl::MutexLock locker(&mutex_);
      auto it = tasks_not_ready_.find(task);
      CHECK(it != tasks_not_ready_.end());
      task_queue_.push_back(it->second);
      tasks_not_ready_.erase(it);
    }

    std::weak_ptr<Task> ThreadPool::Schedule(std::unique_ptr<Task> task)
    {
      std::shared_ptr<Task> shared_task;
      {
        absl::MutexLock locker(&mutex_);
        auto insert_result =
            tasks_not_ready_.insert(std::make_pair(task.get(), std::move(task)));
        CHECK(insert_result.second) << "Schedule called twice";
        shared_task = insert_result.first->second;
      }
      SetThreadPool(shared_task.get());
      return shared_task;
    }

    void ThreadPool::DoWork()
    {
#ifdef __linux__
      // This changes the per-thread nice level of the current thread on Linux. We
      // do this so that the background work done by the thread pool is not taking
      // away CPU resources from more important foreground threads.
      CHECK_NE(nice(10), -1);
#endif
      const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_)
      {
        return !task_queue_.empty() || !running_;
      };
      for (;;)
      {
        std::shared_ptr<Task> task;
        {
          absl::MutexLock locker(&mutex_);
          mutex_.Await(absl::Condition(&predicate));
          if (!task_queue_.empty())
          {
            task = std::move(task_queue_.front());
            task_queue_.pop_front();
          }
          else if (!running_)
          {
            return;
          }
        }
        CHECK(task);
        CHECK_EQ(task->GetState(), common::Task::DEPENDENCIES_COMPLETED);
        Execute(task.get());
      }
    }
  }
} // namespace my_cartographer::common
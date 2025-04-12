//
// Created by whitby on 2025-02-16.
//

#include "my_cartographer/common/thread_pool.h"

#include <absl/memory/memory.h>
#include <gtest/gtest.h>

#include <vector>
#include <iostream>

using namespace my_cartographer::common;

namespace test
{

  class Receiver
  {
  public:
    void Receive(int number)
    {
      absl::MutexLock locker(&mutex_);
      received_numbers_.push_back(number);
      std::cout << "Received number: " << number << std::endl;
    }

    void WaitForNumberSequence(const std::vector<int> &expected_numbers)
    {
      const auto predicate =
          [this, &expected_numbers]() EXCLUSIVE_LOCKS_REQUIRED(mutex_)
      {
        return (received_numbers_.size() >= expected_numbers.size());
      };
      absl::MutexLock locker(&mutex_);
      mutex_.Await(absl::Condition(&predicate));
      EXPECT_EQ(expected_numbers, received_numbers_);
    }

    absl::Mutex mutex_;
    std::vector<int> received_numbers_ GUARDED_BY(mutex_);
  };

  TEST(ThreadPoolTest, RunTask)
  {
    ThreadPool thread_pool(1);
    Receiver receiver;
    auto task = absl::make_unique<Task>();
    task->SetWorkItem([&receiver]()
                      { receiver.Receive(1); });
    thread_pool.Schedule(std::move(task));
    receiver.WaitForNumberSequence({1});
  }

  TEST(ThreadPoolTest, ManyTasks)
  {
    for (int a = 0; a < 5; ++a)
    {
      ThreadPool pool(3);
      Receiver receiver;
      int kNumTasks = 10;
      for (int i = 0; i < kNumTasks; ++i)
      {
        auto task = absl::make_unique<Task>();
        task->SetWorkItem([&receiver]()
                          { receiver.Receive(1); });
        pool.Schedule(std::move(task));
      }
      receiver.WaitForNumberSequence(std::vector<int>(kNumTasks, 1));
    }
  }

  TEST(ThreadPoolTest, RunWithDependency)
  {
    ThreadPool pool(2);
    Receiver receiver;
    auto task_2 = absl::make_unique<Task>();
    task_2->SetWorkItem([&receiver]()
                        { receiver.Receive(2); });
    auto task_1 = absl::make_unique<Task>();
    task_1->SetWorkItem([&receiver]()
                        { receiver.Receive(1); });
    auto weak_task_1 = pool.Schedule(std::move(task_1));
    task_2->AddDependency(weak_task_1);
    pool.Schedule(std::move(task_2));
    receiver.WaitForNumberSequence({1, 2});
  }

  TEST(ThreadPoolTest, RunWithOutOfScopeDependency)
  {
    ThreadPool pool(2);
    Receiver receiver;
    auto task_2 = absl::make_unique<Task>();
    task_2->SetWorkItem([&receiver]()
                        { receiver.Receive(2); });
    {
      auto task_1 = absl::make_unique<Task>();
      task_1->SetWorkItem([&receiver]()
                          { receiver.Receive(1); });
      auto weak_task_1 = pool.Schedule(std::move(task_1));
      task_2->AddDependency(weak_task_1);
    }
    pool.Schedule(std::move(task_2));
    receiver.WaitForNumberSequence({1, 2});
  }

  TEST(ThreadPoolTest, ManyDependencies)
  {
    for (int a = 0; a < 5; ++a)
    {
      ThreadPool pool(5);
      Receiver receiver;
      int kNumDependencies = 10;
      auto task = absl::make_unique<Task>();
      task->SetWorkItem([&receiver]()
                        { receiver.Receive(1); });
      for (int i = 0; i < kNumDependencies; ++i)
      {
        auto dependency_task = absl::make_unique<Task>();
        dependency_task->SetWorkItem([]() {});
        task->AddDependency(pool.Schedule(std::move(dependency_task)));
      }
      pool.Schedule(std::move(task));
      receiver.WaitForNumberSequence({1});
    }
  }

  TEST(ThreadPoolTest, ManyDependants)
  {
    for (int a = 0; a < 5; ++a)
    {
      ThreadPool pool(5);
      Receiver receiver;
      int kNumDependants = 10;
      auto dependency_task = absl::make_unique<Task>();
      dependency_task->SetWorkItem([]() {});
      auto dependency_handle = pool.Schedule(std::move(dependency_task));
      for (int i = 0; i < kNumDependants; ++i)
      {
        auto task = absl::make_unique<Task>();
        task->AddDependency(dependency_handle);
        task->SetWorkItem([&receiver]()
                          { receiver.Receive(1); });
        pool.Schedule(std::move(task));
      }
      receiver.WaitForNumberSequence(std::vector<int>(kNumDependants, 1));
    }
  }

  TEST(ThreadPoolTest, RunWithMultipleDependencies)
  {
    ThreadPool pool(2);
    Receiver receiver;
    auto task_1 = absl::make_unique<Task>();
    task_1->SetWorkItem([&receiver]()
                        { receiver.Receive(1); });
    auto task_2a = absl::make_unique<Task>();
    task_2a->SetWorkItem([&receiver]()
                         { receiver.Receive(2); });
    auto task_2b = absl::make_unique<Task>();
    task_2b->SetWorkItem([&receiver]()
                         { receiver.Receive(2); });
    auto task_3 = absl::make_unique<Task>();
    task_3->SetWorkItem([&receiver]()
                        { receiver.Receive(3); });
    /*          -> task_2a \
     *  task_1 /-> task_2b --> task_3
     */
    auto weak_task_1 = pool.Schedule(std::move(task_1));
    task_2a->AddDependency(weak_task_1);
    auto weak_task_2a = pool.Schedule(std::move(task_2a));
    task_3->AddDependency(weak_task_1);
    task_3->AddDependency(weak_task_2a);
    task_2b->AddDependency(weak_task_1);
    auto weak_task_2b = pool.Schedule(std::move(task_2b));
    task_3->AddDependency(weak_task_2b);
    pool.Schedule(std::move(task_3));
    receiver.WaitForNumberSequence({1, 2, 2, 3});
  }

  TEST(ThreadPoolTest, RunWithFinishedDependency)
  {
    ThreadPool pool(2);
    Receiver receiver;
    auto task_1 = absl::make_unique<Task>();
    task_1->SetWorkItem([&receiver]()
                        { receiver.Receive(1); });
    auto task_2 = absl::make_unique<Task>();
    task_2->SetWorkItem([&receiver]()
                        { receiver.Receive(2); });
    auto weak_task_1 = pool.Schedule(std::move(task_1));
    task_2->AddDependency(weak_task_1);
    receiver.WaitForNumberSequence({1});
    pool.Schedule(std::move(task_2));
    receiver.WaitForNumberSequence({1, 2});
  }
}
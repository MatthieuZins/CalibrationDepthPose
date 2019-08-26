//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Matthieu Zins
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//=========================================================================


#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <functional>
#include <future>
#include <queue>
#include <thread>
#include <vector>

namespace CalibrationDepthPose
{

class ThreadPool
{
public:
  ThreadPool(size_t nbThreads);

  /// Enqueue a new task
  template<class F, class... Args>
  auto enqueue(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>;

  ~ThreadPool();


private:
  std::vector<std::thread> workers;

  std::queue< std::function<void()> > tasks;

  std::mutex queue_mutex;
  std::condition_variable condition;
  std::atomic<bool> stop;
};



template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args)
  -> std::future<typename std::result_of<F(Args...)>::type>
{
  using return_type = typename std::result_of<F(Args...)>::type;

  // Create a new task
  auto task = std::make_shared< std::packaged_task<return_type()> >(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
  );
  std::future<return_type> res = task->get_future();
  {
    // lock other threads
    std::unique_lock<std::mutex> lock(queue_mutex);

    if (stop)
      throw std::runtime_error("Enqueue on stoppped Thread Pool");

    // enqueue the task in the list of tasks
    tasks.emplace([task](){ (*task)(); });
  }

  condition.notify_one();
  return res;
}

} // namespace CalibrationDepthPose

#endif // THREADPOOL_H

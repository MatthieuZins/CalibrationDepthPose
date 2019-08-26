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


#include <CalibrationDepthPose/threadPool.h>

namespace CalibrationDepthPose
{


ThreadPool::ThreadPool(size_t nbThreads) : stop(false)
{
  if (nbThreads == 0)
  {
    nbThreads = std::thread::hardware_concurrency();
  }

  auto workerInternalFunction = [this]() {
    while(true)
    {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(this->queue_mutex);
        this->condition.wait(lock, [this] {
          // the thread stops waiting if stop or if tasks is not empty
          return this->stop || !this->tasks.empty();
        });
        // if stop or no more tasks return
        if (this->stop || this->tasks.empty())
          return ;
        // get next task
        task = std::move(this->tasks.front());
        this->tasks.pop();
      } // implicit lock.unlock();

      // run next task
      task();
    }
  };

  for (size_t i = 0; i < nbThreads; ++i)
  {
    // fill workers with threads executing the same function
    workers.emplace_back(workerInternalFunction);
  }
}

ThreadPool::~ThreadPool()
{
  stop = true;
  condition.notify_all();

  for (auto& worker : workers)
    worker.join();
}


} // namespace CalibrationDepthPose

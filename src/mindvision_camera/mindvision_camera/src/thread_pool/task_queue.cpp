#include "inc/thread_pool/task_queue.hpp"

TaskQueue::Task TaskQueue::takeTask()
{
    Task task;
    std::lock_guard<std::mutex> lockGuard(taskQueueMutex);
    if(!taskQueue.empty())
    {
        task = std::move(taskQueue.front());
        taskQueue.pop();
        return task;
    }
    return nullptr;
}
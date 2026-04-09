#include "inc/thread_pool/thread_pool.hpp"

ThreadPool::ThreadPool(int threadNum) : busyThreadNum(0), shutdown(false), taskQueue(std::make_unique<TaskQueue>())
{
    this->threadNum.store(threadNum);
    for(int i = 0; i < threadNum; ++i)
    {
        threadVec.push_back(std::make_shared<std::thread>(&ThreadPool::worker, this));
        threadVec.back()->detach();
    }
}

void ThreadPool::shut()
{
    {
        std::lock_guard<std::mutex> lk(threadPoolMutex);
        shutdown = true;
        std::cout << "shutdown == ture, in shut()" << std::endl;
    }
    notEmptyCondVar.notify_all();
}

ThreadPool::~ThreadPool()
{
    std::cout << "~ThreadPool() and wait" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void ThreadPool::worker()
{
    while(!shutdown)
    {
        std::unique_lock<std::mutex> uniqueLock(threadPoolMutex);
        notEmptyCondVar.wait(uniqueLock, [this] { return shutdown || !taskQueue->empty(); });
        
        auto taskPtr = taskQueue->takeTask();
        TaskQueue::Task currTask;
        if(taskPtr != nullptr)
        {
            currTask = std::move(taskPtr);
        }
        else 
        {
            std::cout << "get nullptr" << std::endl;
            continue;
        }
        uniqueLock.unlock();    // 尝试一下
        
        ++busyThreadNum;
        currTask();
        --busyThreadNum;
    }
    std::cout << "a worker is finish all task" << std::endl;
}
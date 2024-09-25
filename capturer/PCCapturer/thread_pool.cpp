#include "thread_pool.h"

ThreadPool::ThreadPool()
{
}

void ThreadPool::Init(uint8_t _n_threads, uint8_t _n_jobs)
{
    n_threads = _n_threads; 
    n_jobs = _n_jobs;
    for (uint8_t ii = 0; ii < n_threads; ++ii) {
        threads.emplace_back(std::thread(&ThreadPool::threadWork, this));
    }
}

void ThreadPool::AddJob(const std::function<void()>& job)
{
    std::unique_lock<std::mutex> lock(m);
    /*cv_add.wait(lock, [this] {
        return job_queue.size() <= n_jobs;
    })*/;
    job_queue.push(job);
    cv.notify_one();
}

void ThreadPool::Cleanup()
{
    std::unique_lock<std::mutex> lock(m);
    is_cleaning_up = true;
    cv.notify_all();
}

void ThreadPool::threadWork()
{
    while (true) {
        std::function<void()> job;
        std::unique_lock<std::mutex> lock(m);
        cv.wait(lock, [this] {
            return !job_queue.empty() || is_cleaning_up;
            });
        if (is_cleaning_up) {
            return;
        }
        job = job_queue.front();
        job_queue.pop();
        lock.unlock();
        job();
        //cv_add.notify_one();
    }
}

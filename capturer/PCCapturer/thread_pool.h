#pragma once
#include <cstdint>
#include <queue>
#include <thread>
#include <functional>
#include <condition_variable>
class ThreadPool
{
	public:
		ThreadPool();
		void Init(uint8_t n_threads, uint8_t n_jobs = 100);
		void AddJob(const std::function<void()>& job);
		void Cleanup();
	private:
		uint8_t n_threads = 0;
		uint8_t n_jobs = 0;
		std::mutex m;
		std::condition_variable cv;
		std::condition_variable cv_add;
		std::vector<std::thread> threads;
		std::queue<std::function<void()>> job_queue;
		bool is_cleaning_up = false;

		void threadWork();
};


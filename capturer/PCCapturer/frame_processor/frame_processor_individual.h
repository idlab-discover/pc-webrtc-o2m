#pragma once
#include "frame_processor.h"
#include "../thread_pool.h"
#include <map>
class FrameProcessorIndividual : public FrameProcessor
{
	public:
		FrameProcessorIndividual(std::shared_ptr<Camera> _cam, std::shared_ptr<Transporter> transporter, uint8_t n_threads = 1);
		int ProcessNextFrame();

	private:
		
		std::mutex m;
		std::condition_variable cv;
		uint32_t completed_jobs = 0;
		ThreadPool tp;
};


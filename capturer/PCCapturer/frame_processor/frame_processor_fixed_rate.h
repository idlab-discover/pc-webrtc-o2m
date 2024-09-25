#pragma once
#include "frame_processor.h"
#include "../worker/worker_layered.h"
class FrameProcessorFixedRate : public FrameProcessor
{
	public:
		FrameProcessorFixedRate(std::shared_ptr<Camera> _cam, std::shared_ptr<Transporter> _transporter, std::vector<uint32_t> _targetRates);
		int ProcessNextFrame();
	private:
		std::vector<uint32_t> targetRates;
		std::shared_ptr <WorkerLayered> w_layer;
};


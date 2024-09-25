#pragma once
#include "frame_processor.h"
#include "../worker/worker_layered.h"
class FrameProcessorFixedPcSize : public FrameProcessor
{
	public:
		FrameProcessorFixedPcSize(std::shared_ptr<Camera> _cam, std::shared_ptr<Transporter> _transporter, uint32_t _targetPcSize);
		int ProcessNextFrame();
	private:
		uint32_t targetPcSize;
		std::shared_ptr <WorkerLayered> w_layer;
};


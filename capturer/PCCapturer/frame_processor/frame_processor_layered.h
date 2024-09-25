#pragma once
#include "frame_processor.h"
#include "../camera/camera.h"
#include "../worker/worker_layered.h"
#include "../transporter/transporter.h"

class FrameProcessorLayered : public FrameProcessor
{
	public:
		FrameProcessorLayered(std::shared_ptr<Camera> _cam, std::shared_ptr<Transporter> transporter);
		int ProcessNextFrame();
	private:
		std::shared_ptr <WorkerLayered> w_layer;
};


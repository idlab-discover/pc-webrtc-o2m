#pragma once
#include "../sampler/sampler.h"
#include "../camera/camera.h"
#include "../sampler/sampler_uniform.h"
#include "../transporter/transporter.h"
class FrameProcessor
{
	public:
		FrameProcessor(std::shared_ptr<Camera> _cam, std::shared_ptr<Transporter> transporter);
		virtual int ProcessNextFrame() = 0;
	protected:
		std::shared_ptr<Camera> cam;
		std::shared_ptr<Sampler> sampler;
		std::shared_ptr<Transporter> transporter;
		int curr_frame = -1;

		float predictSampleRate(uint32_t bitrate);
};


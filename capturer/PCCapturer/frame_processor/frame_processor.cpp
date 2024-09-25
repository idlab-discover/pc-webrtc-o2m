#include "frame_processor.h"
FrameProcessor::FrameProcessor(std::shared_ptr<Camera> _cam, std::shared_ptr<Transporter> _transporter) : cam(_cam), transporter(_transporter)
{
}

float FrameProcessor::predictSampleRate(uint32_t estimated_bitrate)
{
    double coeffs[] = { 
        3.4283802218365573,
        -0.5627959277958156,
        0.0455673053725715,
        -0.002040968772146891,
        0.00005123019128697391,
        -0.0000006730896030277754,
        0.000000003596660598793532
    };
    double compression_factor = (double)this->cam->getFrameSize() * 15 / (estimated_bitrate / 8 / 30);
    compression_factor = std::min(50.0, compression_factor);
    double original_factor = compression_factor;
    double sample_rate = coeffs[0];

    for (int j = 1; j < 7; j++) {
        sample_rate += coeffs[j] * compression_factor;
        compression_factor *= original_factor;

    }
    return (float) sample_rate;
}

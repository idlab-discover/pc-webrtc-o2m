#include "frame_processor_layered.h"

FrameProcessorLayered::FrameProcessorLayered(std::shared_ptr<Camera> _cam, std::shared_ptr<Transporter> _transporter) : FrameProcessor(_cam, _transporter)
{
    w_layer = std::make_shared<WorkerLayered>(transporter);
    std::vector<float> layer_ratios = { 0.60, 0.25, 0.15 };
    sampler = std::make_shared<SamplerUniform>(SamplerUniform{ layer_ratios });
}

int FrameProcessorLayered::ProcessNextFrame()
{
    curr_frame++;
    // Capture a frame and save it in the cam object
    // Parameter => perform background removal
    cam->captureFrame(true);
    // Required for result logging
    sampler->setFrameId(curr_frame);
    // Set the current captured frame as the next frame to be sampled
    sampler->setCloud(cam->getCurrentFrame());
    // Perform sampling
    
    sampler->createLayers();
    //    pcl::io::savePLYFileASCII("frame_vop_" + std::to_string(curr_frame) +     ".ply", *cam->getCurrentFrame());
    //std::cout << curr_frame;
    // Get bounding box coordinates of the captured object 
    // Can also include the position of the object here by adding the position to each vector4f
    Eigen::Vector4f min_p;
    Eigen::Vector4f max_p;
    pcl::getMinMax3D(*cam->getCurrentFrame(), min_p, max_p);
    pcl::PointXYZ min_pt(min_p[0], min_p[1], min_p[2]);
    pcl::PointXYZ max_pt(max_p[0], max_p[1], max_p[2]);
    // Multi layer processor will process the different layers by parallel encoding and transmitting them via the selected transporter
    w_layer->process_layers(min_pt, max_pt, std::move(sampler->getLayers()));
	return 0;
}

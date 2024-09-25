#include "frame_processor_fixed_pc_size.h"
FrameProcessorFixedPcSize::FrameProcessorFixedPcSize(std::shared_ptr<Camera> _cam, std::shared_ptr<Transporter> _transporter, uint32_t _targetPcSize) : FrameProcessor(_cam, _transporter), targetPcSize(_targetPcSize)
{
    w_layer = std::make_shared<WorkerLayered>(transporter);
}

int FrameProcessorFixedPcSize::ProcessNextFrame()
{
    curr_frame++;
    cam->captureFrame(true);
  //  if (curr_frame % 100 == 0) {
    //    pcl::io::savePLYFileBinary("frames_up2/frame_" + std::to_string(curr_frame) + ".ply", *cam->getCurrentFrame());
   // }
    
   // std::cout << curr_frame << std::endl;
  //  return 0;
    std::vector<float> layer_ratios = { 0.60, 0.25, 0.15 };
    if (cam->getFrameSize() > targetPcSize) {
        //std::cout << cam->getFrameSize() << " " << targetPcSize << " ";
        for (int i = 0; i < layer_ratios.size(); i++) {
            //std::cout << layer_ratios[i] << " ";
            layer_ratios[i] /= ((float)cam->getFrameSize() / targetPcSize);
            //std::cout << layer_ratios[i] << " ";
        }
        //std::cout << std::endl;
    }

    sampler = std::make_shared<SamplerUniform>(SamplerUniform{ layer_ratios });

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
}
#include "sampler.h"
#include <string>
#include <pcl/point_types.h>
#include <vector>
Sampler::Sampler() {

}
Sampler::Sampler(std::string _file_name) {
    file_name = _file_name;
    input_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile(file_name, *input_cloud);
}
int Sampler::getNumberOfLayers() {
    return number_of_layers;
}
std::string Sampler::getFileName() {
    return file_name;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Sampler::getLayers() {
    return layers;
}

void Sampler::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _input_cloud) {
    input_cloud = _input_cloud;
}
void Sampler::setFrameId(uint32_t _frame_id)
{
    frame_id = _frame_id;
}
int Sampler::getInputCloudSize() {
    return input_cloud->size();
}
std::string Sampler::getLayerFileName(int layer_idx) {
    return getFileName() + std::to_string(layer_idx) + "_" + "_" + getSamplerName() + std::to_string(layer_idx) + ".ply";
}
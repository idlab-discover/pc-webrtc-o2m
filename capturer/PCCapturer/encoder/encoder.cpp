#include "Encoder.h"
void Encoder::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _input_cloud) {
    input_cloud = _input_cloud;
}
void Encoder::setLayerId(uint8_t _layer_id)
{
    layer_id = _layer_id;
}
void Encoder::setFrameId(uint32_t _frame_id)
{
    frame_id = _frame_id;
}
uint8_t Encoder::getLayerId()
{
    return layer_id;
}
uint32_t Encoder::getFrameId()
{
    return frame_id;
}
int Encoder::getInputCloudSize() {
    return input_cloud->size();
}

size_t Encoder::getEncodedSize() {
    return encodedSize;
}
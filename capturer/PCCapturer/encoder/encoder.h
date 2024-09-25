#pragma once
#include <draco/compression/decode.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
class Encoder
{
public:
    void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _input_cloud);
    void setLayerId(uint8_t _layer_id);
    void setFrameId(uint32_t _frame_id);
    int getInputCloudSize();
    uint8_t getLayerId();
    uint32_t getFrameId();
    virtual  const char* encode() = 0;
    size_t getEncodedSize();
protected:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;
    int encodedSize;
    uint8_t layer_id;
    uint32_t frame_id;
};


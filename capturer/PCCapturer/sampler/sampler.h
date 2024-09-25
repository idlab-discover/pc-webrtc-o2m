#pragma once
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/filter_indices.h>
#include <vector>
class Sampler {
public:
    Sampler();
    Sampler(std::string _file_name);
    int getNumberOfLayers();
    std::string getFileName();
    virtual std::string getSamplerName() = 0;
    virtual void createLayers() = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getLayers();
    void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _input_cloud);
    void setFrameId(uint32_t _frame_id);
    int getInputCloudSize();
    std::string getLayerFileName(int layer_idx);
protected:
    std::string sampler_name;
    std::string file_name;
    int number_of_layers;
    uint32_t frame_id;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers;
};
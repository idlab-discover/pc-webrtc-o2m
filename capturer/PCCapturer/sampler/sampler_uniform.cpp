#include "sampler_uniform.h"
#include <string>
#include <vector>
#include <algorithm>
#include <random>
#include <chrono>
#include "../result_writer.h"

SamplerUniform::SamplerUniform(std::vector<float> _layer_ratios) : Sampler() {
    layer_ratios = _layer_ratios;
    number_of_layers = layer_ratios.size();
}
SamplerUniform::SamplerUniform(std::string _file_name, std::vector<float> _layer_ratios) : Sampler(_file_name) {
    layer_ratios = _layer_ratios;
    number_of_layers = layer_ratios.size();
}
std::string SamplerUniform::getSamplerName() {
    std::string name = "uniform_sampler_" + std::to_string(number_of_layers) + "_layers_ratios_";
    for (auto x : layer_ratios) {
        name += std::to_string(x) + "_";
    }
    return name;
}
void SamplerUniform::createLayers() {
    auto start_clock = std::chrono::high_resolution_clock::now();
    layers.clear();
    std::vector<int> pc_points(input_cloud->size());
    std::iota(std::begin(pc_points), std::end(pc_points), 0); // Fill with 0, 1, ..., size -1.
    std::shuffle(pc_points.begin(), pc_points.end(), std::mt19937{std::random_device{}()});

    std::vector<std::vector<int>> layer_indices;
    std::vector<pcl::PointIndices::Ptr> layer_point_indices;
    //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers;
    std::vector<int> layer_sizes;
    std::vector<int> layer_last_indices;
    for (int i = 0; i < number_of_layers; i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr layer(new pcl::PointCloud<pcl::PointXYZRGB>);
        int size = input_cloud->size() * layer_ratios[i];
        layer->reserve(size);
        int last_index = size;
        if (i != 0) {
            last_index += layer_last_indices[i - 1];
        }
        layers.push_back(layer);
        layer_sizes.push_back(size);
        layer_last_indices.push_back(last_index);
    }
    for (int i = 0; i < pc_points.size(); i++) {
        for (int j = 0; j < number_of_layers; j++) {
            if (pc_points[i] > 0 && pc_points[i] < layer_last_indices[j]) {
                layers[j]->push_back(input_cloud->points[i]);
                break;
            }
        }
    }
    //layers.push_back(layer0);
    //layers.push_back(layer1);
    //layers.push_back(layer2);
    auto end_clock = std::chrono::high_resolution_clock::now();
    int duration = duration_cast<std::chrono::milliseconds>(end_clock - start_clock).count();
    //ResultWriter::setLayersPerFrame(frame_id, number_of_layers);
    //ResultWriter::addSamplingDuration(frame_id, duration);
}
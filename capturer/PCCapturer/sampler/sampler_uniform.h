#pragma once
#include "sampler.h"
#include <vector>
#include <string>
class SamplerUniform : public Sampler {
private:
    std::vector<float> layer_ratios;
public:
    SamplerUniform(std::vector<float> _layer_ratios);
    SamplerUniform(std::string _file_name, std::vector<float> _layer_ratios);
    void createLayers();
    std::string getSamplerName();
};
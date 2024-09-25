#pragma once
#include "Encoder.h"
#include <vector>
#include <draco/compression/encode.h>
#include <draco/compression/point_cloud/point_cloud_sequential_encoder.h>
#include <draco/compression/point_cloud/point_cloud_sequential_decoder.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/core/vector_d.h>
#include <draco/core/draco_types.h>
class EncoderDraco :
    public Encoder
{
public:
    const char* encode();
    draco::EncoderBuffer& get_buffer();
private:
    draco::EncoderBuffer buffer;
};


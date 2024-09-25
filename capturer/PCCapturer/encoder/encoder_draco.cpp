#include "encoder_draco.h"
#include <draco/compression/encode.h>
#include <draco/compression/expert_encode.h>
#include <draco/compression/point_cloud/point_cloud_sequential_encoder.h>
#include <draco/compression/point_cloud/point_cloud_sequential_decoder.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/core/vector_d.h>
#include <draco/core/draco_types.h>
#include <string.h>
#include <fstream>
#include <chrono>
#include "../result_writer.h"
#include <random>
const char* EncoderDraco::encode() {
 
    draco::PointCloudBuilder builder;
    draco::PointCloud out_pc;
    builder.Start(input_cloud->size());
    // 3 bytes for position and 1 byte for color
    const int32_t att_id_pos = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    const int32_t att_id_col = builder.AddAttribute(draco::GeometryAttribute::COLOR, 3, draco::DT_UINT8);

    // Set attributes using point cloud values
    for (int i = 0; i < input_cloud->size(); ++i) {
        std::array<float, 3> pos;
        std::array<uint8_t, 3> col;
        pos[0] = input_cloud->points[i].x;
        pos[1] = input_cloud->points[i].y;
        pos[2] = input_cloud->points[i].z;
        
        col[0] = input_cloud->points[i].r;
        col[1] = input_cloud->points[i].g;
        col[2] = input_cloud->points[i].b;
        builder.SetAttributeValueForPoint(att_id_pos, draco::PointIndex(i), &pos);
        builder.SetAttributeValueForPoint(att_id_col, draco::PointIndex(i), &col);
    }
    std::unique_ptr<draco::PointCloud> pc = builder.Finalize(false);

    draco::Encoder encoder;
    encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    // Can potentially change quantization here
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 11);
    encoder.SetSpeedOptions(10, 10);
    encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    std::unique_ptr<draco::ExpertEncoder> expert_encoder;
    expert_encoder.reset(new draco::ExpertEncoder(*pc));
    expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*pc));
    // Best encoding method of the two available
    std::chrono::time_point<std::chrono::high_resolution_clock> start_clock = std::chrono::high_resolution_clock::now();
    if (!expert_encoder.get()->EncodeToBuffer(&buffer).ok()) {
        std::cout << "oops";
    }
    encodedSize = buffer.size();
    
    auto end_clock = std::chrono::high_resolution_clock::now();
    int duration = duration_cast<std::chrono::milliseconds>(end_clock - start_clock).count();
   // std::cout << input_cloud->size() << " " << duration << std::endl;
    //ResultWriter::addEncodingDuration(frame_id, duration, layer_id, input_cloud->size() * 15, encodedSize);
    // Return raw data pointer (easier for further manipulations)
    return buffer.data();
}

draco::EncoderBuffer& EncoderDraco::get_buffer()
{
    return buffer;
}

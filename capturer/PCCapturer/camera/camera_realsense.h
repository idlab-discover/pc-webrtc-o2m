#pragma once
#include "camera.h"

// Realsense camera class
class CameraRealsense : public Camera
{
public:
    CameraRealsense(int width = 848, int height = 484, int fps = 30);
    void captureFrame(bool filter_background);

private:
    rs2::pointcloud pc;
    rs2::pipeline pipe;
    rs2::config cfg;
    int rgbTexture(int width, int height, int bpp, int sib, rs2::texture_coordinate Texture_XY);
    cloud_pointer pclConversion(const rs2::points& points, const rs2::video_frame& color);

};


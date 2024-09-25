#pragma once
#include <string>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/filter_indices.h>
#include <vector>
// Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <Windows.h>
typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef point_cloud::Ptr prevCloud;

class Camera {
public:
    Camera(int width = 848, int height = 484, int fps = 30);
    cloud_pointer getCurrentFrame();
    virtual void captureFrame(bool filter_background) = 0;
    int getFrameSize();
    void setFramesToSkip(int _frames_to_skip);
protected:
    int frames_to_skip = 0;
    int target_fps;
    cloud_pointer current_frame;
};
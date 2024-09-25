#include "camera_Realsense.h"
#include <string>
#include <pcl/point_types.h>
#include <windows.h>
#include <vector>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
// Intel Realsense Headers
#include <librealsense2/rs.hpp> 
// Based on https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl-color/rs-pcl-color.cpp
// Optimized to reduce latency
//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
int CameraRealsense::rgbTexture(int width, int height, int bpp, int sib, rs2::texture_coordinate Texture_XY)
{


    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(Texture_XY.u * width + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * bpp;   // Get # of bytes per pixel
    int strides = y_value * sib; // Get line width in bytes
    int Text_Index = (bytes + strides);



    // RGB components to save in tuple

    return Text_Index;
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 
cloud_pointer CameraRealsense::pclConversion(const rs2::points& points, const rs2::video_frame& color) {
    // Timer::startTimer();
     // Object Declaration (Point Cloud)
    cloud_pointer cloud(new point_cloud);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    int  RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();
    // Timer::endTimer();
     //std::cout << "Time Ubut" << Timer::getDuration() << "ms" << std::endl;
     // Iterating through all points and setting XYZ coordinates
     // and RGB values
    // Timer::startTimer();
     // Get Width and Height coordinates of texture
    int width = color.get_width();  // Frame width in pixels
    int height = color.get_height(); // Frame height in pixels
    int bpp = color.get_bytes_per_pixel();
    int sib = color.get_stride_in_bytes();
    const auto New_Texture = reinterpret_cast<const uint8_t*>(color.get_data());
    for (int i = 0; i < points.size(); i++)
    {
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = rgbTexture(width, height, bpp, sib, Texture_Coord[i]);
        int NT1 = New_Texture[RGB_Color];
        int NT2 = New_Texture[RGB_Color + 1];
        int NT3 = New_Texture[RGB_Color + 2];
        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = (NT1); // Reference tuple<2>
        cloud->points[i].g = (NT2); // Reference tuple<1>
        cloud->points[i].b = (NT3); // Reference tuple<0>

    }
    //  Timer::endTimer();
      //std::cout << "Time Loop" << Timer::getDuration() << "ms" << std::endl;
    return cloud; // PCL RGB Point Cloud generated
}

CameraRealsense::CameraRealsense(int width, int height, int fps) : Camera(width, height, fps) {
    // Enable the required streams
    cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 848, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);


    rs2::pipeline_profile selection = pipe.start(cfg);
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        pipe.wait_for_frames();
        //  depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }

    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        
        Sleep(1);
        std::cout << "laser power " << range.max << std::endl;
        //depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }
    //depth_sensor.set_option(RS2_OPTION_MAX_DISTANCE, 0.75);
}

void CameraRealsense::captureFrame(bool filter_background) {
   
    // Capture a single frame and obtain depth + RGB values from it
    if (frames_to_skip > 0) {
        for (int i = 0; i < frames_to_skip; i++) {
            pipe.wait_for_frames();
        }
    }
  
    auto frames = pipe.wait_for_frames();

    auto depth = frames.get_depth_frame();
    auto RGB = frames.get_color_frame();
    // Map Color texture to each point
    pc.map_to(RGB);
    // Generate Point Cloud
    auto points = pc.calculate(depth);
    
    // Convert generated Point Cloud to PCL Formatting
    cloud_pointer cloud = pclConversion(points, RGB);

    // Filter point cloud using z filter
    if (filter_background) {
        /*pcl::PassThrough<pcl::PointXYZRGB> cloud_filter; // Create the filtering object
        cloud_filter.setInputCloud(cloud);           // Input generated cloud to filter
        cloud_filter.setFilterFieldName("z");        // Set field name to Z-coordinate
        cloud_filter.setFilterLimits(0.0, 1.0);     // Set accepted interval values
        cloud_filter.filter(*current_frame);    */          // Filtered Cloud Outputted

        pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter(true);
        cropBoxFilter.setInputCloud(cloud);
        Eigen::Vector4f min_pt(-0.7f, -1.0f, 0.0f, 1.0f);
        Eigen::Vector4f max_pt(0.7f, 1.0f, 1.0f, 1.0f);
        cropBoxFilter.setMin(min_pt);
        cropBoxFilter.setMax(max_pt);

        cropBoxFilter.filter(*current_frame);
    }
    else {
        current_frame = cloud;
    }
    //std::cout << current_frame->size() << std::endl;
}
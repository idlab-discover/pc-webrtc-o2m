#pragma once
#include "Camera.h"
#include <thread>
#include <fstream>
#include <filesystem>
#include <pcl/io/ply_io.h>
// Mock Camera 
// Will use .ply files instead of captured frames
class CameraMock : public Camera
{
public:
	CameraMock(std::string _directory, int width = 848, int height = 484, int fps = 30);
	void captureFrame(bool filter_background);
private:
	std::vector<cloud_pointer> frames;
	// Counter that will be used to go over the sequence of files in a round robin fashion
	int rr_counter = 0;
	int n_files = 0;
	std::string directory;
	std::chrono::milliseconds interframe_delay;
	std::chrono::steady_clock::time_point previous_time;

};


#include "camera_mock.h"
CameraMock::CameraMock(std::string _directory, int width, int height, int fps) : Camera(width, height, fps) {
    target_fps = fps;
    interframe_delay = std::chrono::milliseconds(1000 / target_fps);
    previous_time = std::chrono::high_resolution_clock::now();
    directory = _directory;
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    try {
        for (const auto& entry : std::filesystem::directory_iterator(directory)) {
            if (std::filesystem::is_regular_file(entry)) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::io::loadPLYFile(entry.path().string(), *cloud);
                frames.push_back(std::move(cloud));
            }
            n_files++;
        }
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

}
void CameraMock::captureFrame(bool filterBackground) {
    std::cout << "";
    current_frame = frames[rr_counter];
    // Need this function to enable precise sleep
    timeBeginPeriod(1);
    auto current_time = std::chrono::high_resolution_clock::now(); // Get the end time of the loop
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - previous_time); // Calculate the elapsed time in milliseconds
    previous_time = current_time;
    if (elapsed_time < interframe_delay) // If the elapsed time is less than the desired frame time, sleep for the remaining time
    {
        //std::cout << interframe_delay - elapsed_time << std::endl;
        std::this_thread::sleep_for(interframe_delay - elapsed_time);
    }
    previous_time = std::chrono::high_resolution_clock::now();
    // Need to call end here for optimisation
    timeEndPeriod(1);
    rr_counter = (rr_counter + 1) % n_files;
}


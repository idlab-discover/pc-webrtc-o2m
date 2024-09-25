#include <librealsense2/rs.hpp>
#define WIN32_LEAN_AND_MEAN 

// PCL 
#include <pcl/common/io.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <filesystem>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <chrono>
#include "sampler/sampler_uniform.h"
#include "encoder/encoder_draco.h"
#include "transporter/transporter_udp_proxy.h"
//#include "WebRTCTransporter.h"
#include <Windows.h>
#include <winsock2.h>
//#include "WebRTCTransporter.h"
#include "result_writer.h"
#include "camera/camera_realsense.h"
#include "camera/camera_mock.h"
#include "frame_processor/frame_processor_layered.h"
#include "frame_processor/frame_processor_individual.h"
#include "frame_processor/frame_processor_fixed_rate.h"
#include "frame_processor/frame_processor_fixed_pc_size.h"
#include "transporter/transporter_mock.h"
#pragma comment(lib,"ws2_32.lib") //Winsock Library

// Global Variables
std::string cloudFile; // .pcd file name
std::string prevCloudFile; // .pcd file name (Old cloud)
int i = 1; // Index for incremental file name

// Code to parse command line parameters
class InputParser {
public:
    InputParser(int& argc, char** argv) {
        for (int i = 1; i < argc; ++i)
            this->tokens.push_back(std::string(argv[i]));
    }
    /// @author iain
    const std::string& getCmdOption(const std::string& option) const {
        std::vector<std::string>::const_iterator itr;
        itr = std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end()) {
            return *itr;
        }
        static const std::string empty_string("");
        return empty_string;
    }
    /// @author iain
    bool cmdOptionExists(const std::string& option) const {
        return std::find(this->tokens.begin(), this->tokens.end(), option)
            != this->tokens.end();
    }
private:
    std::vector <std::string> tokens;
};


//#define SAVE_RESULTS
int main(int argc, char** argv)
{
    InputParser input(argc, argv);

    std::shared_ptr<Transporter> transporter(nullptr);
    std::shared_ptr<Camera> cam(nullptr);
    std::shared_ptr<FrameProcessor> frame_proc(nullptr);

    // ------ Command line argument processing --------
    const std::string& results_file = input.getCmdOption("-r");
    bool save_results = false;
    std::cout << results_file << " file";
    if (!results_file.empty()) {
        ResultWriter::setFileName(results_file);
        std::cout << results_file;
    }
    //ResultWriter::init();

    bool use_mock_transport = false;
    if (input.cmdOptionExists("-t")) {
        use_mock_transport = true;
    }

    std::string proxy_config = std::string(input.getCmdOption("-p"));
    if (proxy_config.empty() && !use_mock_transport) {
        std::cout << "Provide proxy config with -p";
        return 0;
    }

    bool use_camera = false;
    std::string content_dir;
    if (input.cmdOptionExists("-i")) {
        use_camera = true;
    }
    else {
        content_dir = std::string(input.getCmdOption("-d"));
        if (content_dir.empty()) {
            std::cout << "Use -d to provide a directory with .ply files when not using a RealSense camera";
            return 0;
        }
    }

    // Use layered encoding vs individual encoding
    // Server needs to also use this option
    const std::string& processor_strategy = input.getCmdOption("-l");
    if (processor_strategy.empty()) {
        std::cout << "Use -l to provide a processor strategy (layer, indiv, fixed)";
    }
    // END ------ Command line argument processing --------
    //std::vector<float> layer_ratios = { 0.60, 0.25 };
    //std::shared_ptr<Sampler> sampler = std::make_shared<SamplerUniform>(SamplerUniform{ layer_ratios });


    // ------ Network tranport initiliasation ------
    std::cout << "transporter";
    if (use_mock_transport) {
        transporter = std::make_shared<TransporterMock>();
    }
    else {
        transporter = std::make_shared<TransporterUDPProxy>();
    }
    
    transporter->Init(proxy_config);
    transporter->SetupConnection();
    while (!transporter->GetIsClientConnected()) {
        // Busy Wait, maybe change to condition variable
    }
    std::cout << "can start sending data" << std::endl;


    // END ------ Network tranport initiliasation ------
    // ------ Capturing & Processing Loop ------
    //MultiLayerProcessor m(transporter);
    int curr_frame = -1;
    if (use_camera) {
        cam = std::make_shared<CameraRealsense>();
    }
    else {
        cam = std::make_shared<CameraMock>(content_dir);
    }

    if(processor_strategy == "layer") {
        frame_proc = std::make_shared<FrameProcessorLayered>(cam, transporter);
    } 
    else if (processor_strategy == "indiv") {
        uint8_t n_threads = 3;
        auto indi_proc = std::make_shared<FrameProcessorIndividual>(cam, transporter, n_threads);
        frame_proc = indi_proc;
    }
    else if (processor_strategy == "fixed") {
        frame_proc = std::make_shared<FrameProcessorFixedRate>(cam, transporter, std::vector<uint32_t>{35000000, 25000000, 15000000});
    }
    else if (processor_strategy == "fixed_pc") {
        frame_proc = std::make_shared<FrameProcessorFixedPcSize>(cam, transporter, 125000);
    }
    else {
        std::cout << "Invalid processor strategy" << std::endl;
        return 0;
    }

    while (true) {
        frame_proc->ProcessNextFrame();
    }
    // END ------ Capturing & Processing Loop ------
    std::cout << "Exiting Program... " << std::endl;
    return EXIT_SUCCESS;
}

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include "../transporter/transporter.h"
#include "../encoder/encoder_draco.h"

struct MultiLayerMainHeader {
	uint32_t n_layers;
	float min_x;
	float min_y;
	float min_z;
	float max_x;
	float max_y;
	float max_z;
};
struct MultiLayerSideHeader {
	uint32_t layer_id;
	uint32_t framelen;
};

class WorkerLayered
{
public:
	WorkerLayered(std::shared_ptr<Transporter> _transporter);
	void process_layers(pcl::PointXYZ min, pcl::PointXYZ max, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>&& layers);
private:
	std::queue<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> layers_buffer;
	std::queue<pcl::PointXYZ> min_buffer;
	std::queue<pcl::PointXYZ> max_buffer;
	std::shared_ptr<Transporter> transporter;
	std::thread wrk;
	//std::vector<std::thread> encoding_wrk;
	std::mutex m;
	std::condition_variable cv;
	std::vector<EncoderDraco> encoders;
	uint32_t current_frame = 0;
	void processing_work();
	void encoding_work(pcl::PointCloud<pcl::PointXYZRGB>::Ptr l, uint8_t layer_id);
};


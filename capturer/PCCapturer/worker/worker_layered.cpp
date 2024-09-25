#include "worker_layered.h"

WorkerLayered::WorkerLayered(std::shared_ptr<Transporter> _transporter)
{
	transporter = _transporter;
	// Need to create seperate thread for all processing work
	// Main thread can still continue capturing / sampling etc while this thread encodes / transmits the layers
	wrk = std::thread(&WorkerLayered::processing_work, this);
}

void WorkerLayered::process_layers(pcl::PointXYZ min, pcl::PointXYZ max, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>&& layers)
{
	std::unique_lock<std::mutex> lock(m);
	// Want to fill all buffers before giving control to the processing thread
	layers_buffer.push(layers);
	min_buffer.push(min);
	max_buffer.push(max);
	lock.unlock();
	// Notify the worker thread
	cv.notify_all();
}

void WorkerLayered::processing_work()
{
	while (true) {
		std::unique_lock<std::mutex> lock(m);
		cv.wait(lock);
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ls = layers_buffer.front();
		pcl::PointXYZ min_pt = min_buffer.front();
		pcl::PointXYZ max_pt = max_buffer.front();
		layers_buffer.pop();
		min_buffer.pop();
		max_buffer.pop();
		// Once the data is read / removed from the buffers we can release the lock
	

		// -------- Encoding --------
		// Start seperate thread for each of the layers
		std::vector<std::thread> encoding_wrk;
		uint8_t layer_id = 0;
		for (auto& l : ls) {
			encoding_wrk.push_back(std::thread(&WorkerLayered::encoding_work, this, l, layer_id));
			layer_id++;
		}
		// Wait until all threads have encoded their layers
		for (auto& t : encoding_wrk) {
			t.join();
		}
		lock.unlock();
		// END -------- Encoding --------
		// -------- Transport --------
		// Main header contains the bounding box of the PC
		struct MultiLayerMainHeader m { encoders.size(), min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z };
		uint32_t full_size = sizeof(m);
		int ll = 0;
		for (auto& e : encoders) {
			full_size += sizeof(MultiLayerSideHeader) + e.getEncodedSize();
			ll++;
		}
		std::vector<char> backing_data;
		backing_data.resize(full_size);
		uint32_t offset = sizeof(m);
		memcpy(backing_data.data(), &m, sizeof(m));
		// Iterate over each encoder (in this case a draco encoder)
		// Encoders are not sorted so it is possible that the layers are out of order here (does not matter for rate allocation algorithm)
		// TODO: potentially sort them / insert data at "correct" position
		for (auto& e : encoders) {
			struct MultiLayerSideHeader s { e.getLayerId(), e.getEncodedSize() };
			memcpy(backing_data.data() + offset, &s, sizeof(s));
			offset += sizeof(s);
			memcpy(backing_data.data() + offset, e.get_buffer().data(), e.getEncodedSize());
			offset += e.getEncodedSize();
		}
		transporter->SendEncodedData(full_size, backing_data.data(), false, 0);
		// END -------- Transport --------

		encoders.clear();
		current_frame++;

	}
}

void WorkerLayered::encoding_work(pcl::PointCloud<pcl::PointXYZRGB>::Ptr l, uint8_t layer_id)
{
	EncoderDraco d;
	d.setLayerId(layer_id);
	d.setCloud(l);
	d.setFrameId(current_frame);
	const char* encoded_data = d.encode();
	encoders.push_back(std::move(d));
}

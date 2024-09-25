#include "result_writer.h"
std::map<uint32_t, std::vector<ResultRecord>> ResultWriter::results;
std::map<uint32_t, uint8_t> ResultWriter::layers_per_frame;
std::queue<ResultRecord> ResultWriter::results_queue;
std::mutex ResultWriter::m;
std::mutex ResultWriter::m_queue;
std::thread ResultWriter::wrk;
std::condition_variable ResultWriter::cv;
std::string ResultWriter::fileName;


void ResultWriter::init()
{
	// Need a seperate thread to reduce IO overhead to the main thread
	wrk = std::thread(&ResultWriter::save_results_work);
}

// Duration of sampling
void ResultWriter::addSamplingDuration(uint32_t frame_id, uint32_t duration)
{
	if (frame_id % SAVE_FRAME_N != 0)
		return;
	std::lock_guard<std::mutex> lock(m);
	addRecordIfNotExist(frame_id);
	for (int i = 0; i < layers_per_frame[frame_id]; i++) {
		results[frame_id][i].sampling = duration;
	}
}

// Duration of Encoding (layer based)
void ResultWriter::addEncodingDuration(uint32_t frame_id, uint32_t duration, uint8_t layer_id, uint32_t layer_size, uint32_t encoded_size)
{
	if (frame_id % SAVE_FRAME_N != 0)
		return;
	std::lock_guard<std::mutex> lock(m);
	addRecordIfNotExist(frame_id);
	results[frame_id][layer_id].encoding = duration;
	results[frame_id][layer_id].layer_id = layer_id;
	results[frame_id][layer_id].layer_size = layer_size;
	results[frame_id][layer_id].encoded_size = encoded_size;
}

// Timestamp of frame transmission
void ResultWriter::addSendTimestamp(uint32_t frame_id, uint8_t layer_id, uint64_t _full_size)
{
	if (frame_id % SAVE_FRAME_N != 0)
		return;
	std::lock_guard<std::mutex> lock(m);
	addRecordIfNotExist(frame_id);
	std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()
	);
	//for (int i = 0; i < layers_per_frame[frame_id]; i++) {
		results[frame_id][layer_id].send_timestamp = ms.count();
		results[frame_id][layer_id].full_size = _full_size;
	//}

}

// Method that will mark the record as ready and add it to a buffer
void ResultWriter::setFrameReadyToSave(uint32_t frame_id)
{
	if (frame_id % SAVE_FRAME_N != 0)
		return;
	std::lock_guard<std::mutex> lock(m);
	std::unique_lock<std::mutex> lock_queue(m_queue);
	for (int i = 0; i < layers_per_frame[frame_id]; i++) {
		results_queue.push(std::move(results[frame_id][i]));
	}
	results.erase(frame_id);
	lock_queue.unlock();
	cv.notify_all();
}

void ResultWriter::setLayersPerFrame(uint32_t frame_id, uint8_t layers)
{
	if (frame_id % SAVE_FRAME_N != 0)
		return;
	layers_per_frame[frame_id] = layers;
	addRecordIfNotExist(frame_id);
}

// This will add a record for the frame if it doesn't already exist
void ResultWriter::addRecordIfNotExist(uint32_t frame_id)
{
	if (frame_id % SAVE_FRAME_N != 0)
		return;
	auto it = results.find(frame_id);
	if (it == results.end()) {
		for (int i = 0; i < layers_per_frame[frame_id]; i++) {
			results[frame_id].push_back(ResultRecord{ frame_id,0,0,0,0,0,0,0 });
		}
	}
}

// Work thread for saving the results to a file
void ResultWriter::save_results_work()
{
	std::ofstream results_file(fileName, std::ios::trunc);
	results_file << "frame_nr;sampler_time;layer_id;layer_size;encoding_time;encoded_size;send_timestamp;full_size" << std::endl;
	while (true) {
		std::unique_lock<std::mutex> lock(m_queue);
		cv.wait(lock);
		while (!results_queue.empty()) {
			auto r = results_queue.front();
			results_queue.pop();
			results_file << r.frame_id << ";" << r.sampling << ";" << r.layer_id << ";" << r.layer_size << ";" << r.encoding << ";" << r.encoded_size << ";" << r.send_timestamp << ";" << r.full_size << std::endl;
		}
		lock.unlock();
	}
}
void ResultWriter::setFileName(std::string _fileName) {
	fileName = _fileName;
}

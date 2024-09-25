#pragma once
#include <cstdint>
#include <map>
#include <chrono>
#include <mutex>
#include <queue>
#include <fstream>
#define SAVE_FRAME_N 5
struct ResultRecord {
	uint32_t frame_id; // Automatic
	uint32_t sampling; // Set in sampling
	uint32_t encoding; // Set in encoding
	uint32_t layer_id; // Set in encoding
	uint32_t layer_size; // Set in encoding
	uint32_t encoded_size; // Set in encoding
	uint64_t full_size; // Set in transporter
	uint64_t send_timestamp; // Set in transporter
};

class ResultWriter
{
public:
	static void init();
	static void addSamplingDuration(uint32_t frame_id, uint32_t duration);
	static void addEncodingDuration(uint32_t frame_id, uint32_t duration, uint8_t layer_id, uint32_t layer_size, uint32_t encoded_size);
	static void addSendTimestamp(uint32_t frame_id, uint8_t layer_id, uint64_t full_size);
	static void setFrameReadyToSave(uint32_t frame_id);
	static void setLayersPerFrame(uint32_t frame_id, uint8_t layers);
	static void setFileName(std::string fileName);
private:
	static std::map<uint32_t, std::vector<ResultRecord>> results;
	static std::map<uint32_t, uint8_t> layers_per_frame;
	static std::queue<ResultRecord> results_queue;
	static std::mutex m;
	static std::mutex m_queue;
	static std::thread wrk;
	static std::condition_variable cv;
	static std::string fileName;
	static bool save_files;
	static void addRecordIfNotExist(uint32_t frame_id);
	static void save_results_work();

};


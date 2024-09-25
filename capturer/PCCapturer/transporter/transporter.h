#pragma once
#include <vector>
#include <string>
#include <mutex>
#include <iostream>
#include "../result_writer.h"

enum PacketType {
	ReadyPacketType   = 0,
	FramePacketType   = 1,
	AudioPacketType   = 2,
	ControlPacketType = 3
};

struct MessageType {
	uint8_t packet_type;
};
struct ConnectionData {
	MessageType msg_type;
	uint32_t extra_data_len;
};
struct VideoPacketHeader {
	uint32_t client_d;
	uint32_t frame_nr;
	uint32_t frame_len;
	uint32_t frame_offset;
	uint32_t packet_len;
};
struct BitrateRequestHeader {
};
struct InputPacket {
	uint32_t status;
	size_t packet_len;
	char* data;
};
class Transporter
{
public:
	virtual void Init(const std::string& config_path) = 0;
	virtual void SetupConnection() = 0;
	virtual void SendEncodedData(size_t size, const char* data, bool indi, uint64_t clientID) = 0;
	virtual InputPacket PollNextPacket(uint32_t buf_size) = 0;
	virtual std::map<uint64_t, uint32_t> GetClientBitrates() = 0;
	bool GetIsClientConnected();
protected:
	bool isClientConnected;
	std::map<uint32_t, uint32_t> frame_counters;
	std::vector<char> input_buffer;
	std::vector<char> output_buffer;

	std::mutex m;
};


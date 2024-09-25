#pragma once
#include <cstdint>
#include <queue>
class ReceivedFrame {
public:
	ReceivedFrame() {}
	ReceivedFrame(uint32_t _framelen, uint32_t _framenr) {
		framelen = _framelen;
		framenr = _framenr;
		current_size = 0;
		data.resize(framelen);
	}
	~ReceivedFrame() {
	}
	bool insert(char* b, uint32_t frameoffset, uint32_t len, size_t tt) {
		if (len == 0)
			return true;
		std::memcpy(&data[frameoffset], b, len);
		current_size += len;
		return true;
	}
	bool is_complete() {
		return current_size == framelen;
	}

	uint32_t get_framenr() const {
		return framenr;
	}
	uint32_t get_framelen() {
		return framelen;
	}
	uint32_t get_current_size() {
		return current_size;
	}

	char* get_data() {
		return data.data();
	}
	size_t get_data_length() {
		return data.size();
	}
	std::vector<char> get_data_v() {
		return  data;
	}
	uint8_t get_temp() {
		return temp;
	}
private:
	uint32_t framelen = 0;
	uint32_t framenr = 0;
	uint32_t current_size = 0;
	std::vector<char> data;
	uint8_t temp = 0;
};
bool operator<(const ReceivedFrame& f1, const ReceivedFrame& f2)
{
	return f1.get_framenr() > f2.get_framenr();
}
#include <cstdint>
#include <cstring>
#include "ReceivedFrame.hpp"

class DataParser {
public:
	DataParser() {}
	void fill_data_array(void* d) {
		char * p = current_frame.get_data();
		uint32_t framelen = current_frame.get_framelen();
		char* temp_d = reinterpret_cast<char*>(d);
		memcpy(temp_d, p, framelen);
	}
	void set_current_frame(ReceivedFrame& r) {
		current_frame = std::move(r);
	}
	uint32_t get_current_frame_size() {
		return current_frame.get_framelen();
	}
private:
	ReceivedFrame current_frame;
};
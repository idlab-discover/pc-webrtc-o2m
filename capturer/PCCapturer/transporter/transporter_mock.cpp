#include "transporter_mock.h"

void TransporterMock::Init(const std::string& config_path) {
	
}

// Connection setup performed by sending / receiving a simple hello message
void TransporterMock::SetupConnection() {
	isClientConnected = true;
}

void TransporterMock::SendEncodedData(size_t size, const char* encoded_pc, bool indi, uint64_t clientID) {
	std::unique_lock lk(m);
	if (frame_counter % 100 == 0)
	{
		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()
		);
		std::cout << "Frame Complete " << frame_counter << " TS=" << ms.count() << std::endl;
		// Remove newly added frame to make sure that memory doesn't get full
	}
	//ResultWriter::addSendTimestamp(frame_counter, 0, size);
	//ResultWriter::setFrameReadyToSave(frame_counter);
	frame_counter++;
	return;
}

InputPacket TransporterMock::PollNextPacket(uint32_t buf_size)
{
	return InputPacket(0);
}

std::map<uint64_t, uint32_t> TransporterMock::GetClientBitrates() {
	std::map<uint64_t, uint32_t> client_to_bw;
    for (int i = 0; i < 5; i++) {
        client_to_bw[i] = rand() % (100000000 - 20000000 + 1) + 20000000;
    }
	return client_to_bw;
}
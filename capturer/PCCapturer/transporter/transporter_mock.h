#pragma once
#include "transporter.h"
class TransporterMock : public Transporter
{
    public:
        void Init(const std::string& config_path);
        void SetupConnection();
        void SendEncodedData(size_t size, const char* data, bool indi, uint64_t clientID);
        InputPacket PollNextPacket(uint32_t buf_size);
        std::map<uint64_t, uint32_t> GetClientBitrates();
    private:
        int frame_counter = 0;
};


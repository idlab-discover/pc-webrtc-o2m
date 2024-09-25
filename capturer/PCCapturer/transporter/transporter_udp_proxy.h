#pragma once
#include "transporter.h"
#include <stdio.h>
#include <winsock2.h>
#include <Ws2tcpip.h>
#include <iostream>
#include <chrono>
#include <memory.h>
#include <map>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include <windows.h>

struct ProxyConfig {
    uint32_t self_port;
    std::string self_addr;
    uint32_t srv_port;
    std::string srv_addr;
    std::string srv_path;
    bool is_indi;
};

class TransporterUDPProxy : public Transporter
{
public:
    void SetFragmentSize(int _fragmentSize);
    void Init(const std::string& config_path);
    void SetupConnection();
    void SendEncodedData(size_t size, const char* data, bool indi, uint64_t clientID);
    std::map<uint64_t, uint32_t> GetClientBitrates();
    InputPacket PollNextPacket(uint32_t buf_size);
private:
    WSADATA wsa;
    WSAPOLLFD pollfds[1];
    struct sockaddr_in si_other;
    struct sockaddr_in si_self;
    ProxyConfig config;
    int s, slen = sizeof(si_other);

    void StartPeerProcess(
        const std::string& peer_executable_path,
        const std::string& capturer_addr,
        const std::string& svr_addr,
        bool peer_in_window,
        bool peer_dont_close,
        const std::string& name,
        bool is_indi
    );
    int SendPacket(char* data, uint32_t size, uint32_t _packet_type);
};


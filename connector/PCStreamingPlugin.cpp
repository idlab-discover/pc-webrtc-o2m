// PCStreamingPlugin.cpp : Defines the exported functions for the DLL.
//

#include "pch.h"
#include "framework.h"
#include "PCStreamingPlugin.h"
#include "packet_data_defs.hpp"
#include "FrameBuffer.hpp"
#include "DataParser.hpp"
#define SERVER "172.22.107.250"	//ip address of udp server
#define BUFLEN 1300	//Max length of buffer
#define PORT 8000	//The port on which to listen for incoming data

static int sum = 0;
static struct sockaddr_in si_other;
static int s, slen = sizeof(si_other);
static char* buf = (char*)malloc(BUFLEN);
static char* buf_ori = buf;
static WSADATA wsa;
static SOCKET s_send;
static struct sockaddr_in si_send;
static struct sockaddr_in our_address;
int slen_send = sizeof(si_send);
static std::thread wrk;
static std::map<uint32_t, ReceivedFrame> recv_frames;
static FrameBuffer frame_buffer;
static DataParser data_parser;
static int p_c = 0;
static bool keep_working = false;
static std::string server = "172.22.107.250";
static std::condition_variable cv;

static std::mutex m_recv_data;
static std::mutex m_buff;
enum CONNECTION_SETUP_CODE : int
{
    ConnectionSuccess = 0,
    StartUpError = 1,
    SocketCreationError = 2,
    SendToError = 3
};

enum PACKET_TYPE {
    PeerReadyPacket = 0,
    FramePacket = 1,
    AudioPacket = 2,
    ControlPacket = 3
};

int setup_connection(char* server_str, uint32_t server_port, uint32_t self_port) {
    frame_buffer.reset();
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        return StartUpError;
    }

    keep_working = true;
    buf = (char*)malloc(BUFLEN);
    buf_ori = buf;
    // Generic parameters
    ULONG buf_size = 524288000;

    // Create send socket
    if ((s_send = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
        WSACleanup();
        return SocketCreationError;
    }

    // Binding ports
    
    our_address.sin_family = AF_INET;
    our_address.sin_port = htons(self_port);
    our_address.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(s_send, (struct sockaddr*)&our_address, sizeof(our_address)) < 0) {
       
    }

    // Set socket options
    if (setsockopt(s_send, SOL_SOCKET, SO_RCVBUF, (char*)&buf_size, sizeof(ULONG)) < 0) {
      
    }
    si_send.sin_family = AF_INET;
    si_send.sin_port = htons(server_port);
    inet_pton(AF_INET, server_str, &si_send.sin_addr.S_un.S_addr);


    sockaddr_in our_addr;
    socklen_t our_addr_len = sizeof(our_addr);
    getsockname(s_send, (sockaddr*)&our_addr, &our_addr_len);
    return ConnectionSuccess;
}

void listen_work() {
    // TODO: add poll for performance, maybe
    while (keep_working) {
        std::unique_lock<std::mutex> guard(m_recv_data);
        size_t size = 0;
        // Receive from the only available socket
        if ((size = recvfrom(s_send, buf, BUFLEN, 0, NULL, NULL)) == SOCKET_ERROR) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        struct PacketType p_type(&buf, size);
        // Check if frame=>0 or control=>1
        switch (p_type.type) {
            case (PeerReadyPacket): {
                char t[BUFLEN] = { 0 };
                if (sendto(s_send, t, BUFLEN, 0, (struct sockaddr*)&si_send, slen_send) == SOCKET_ERROR) {
                    WSACleanup();
                    return;
                }
                break;
            }
            case (FramePacket): {
                // Parse frame packet
                struct PacketHeader p_header(&buf, size);
                auto frame = recv_frames.find(p_header.framenr);
                if (frame == recv_frames.end()) {
                    auto e = recv_frames.emplace(p_header.framenr, ReceivedFrame(p_header.framelen, p_header.framenr));
                    frame = e.first;
                }
                frame->second.insert(buf, p_header.frameoffset, p_header.packetlen, size);
                if (frame->second.is_complete()) {
                    m_buff.lock();
                    frame_buffer.insert_frame(frame->second);
                    recv_frames.erase(p_header.framenr);
                    m_buff.unlock();
                    cv.notify_all();

                }
                break;
            }
        };
        if (p_type.type == FramePacket) {
            
        }
        buf = buf_ori;
        guard.unlock();
    }
}

void start_listening() {
    wrk = std::thread(listen_work);
}

int next_frame() {
    std::unique_lock<std::mutex> lock(m_buff);
    cv.wait(lock, []() {
        return !keep_working || frame_buffer.get_buffer_size() > 0;
    });
    if (!keep_working) {
        return -1;
    }
    ReceivedFrame f = frame_buffer.next();
    data_parser.set_current_frame(f);
    return data_parser.get_current_frame_size();
}
void set_data(void* d) {
    data_parser.fill_data_array(d);
}

void clean_up() {
    keep_working = false;
    closesocket(s_send);
    if (wrk.joinable())
        wrk.join();
    if (buf != NULL) {
        free(buf);
        buf = NULL;
    }
    recv_frames.clear();
    cv.notify_all();
    //WSACleanup();
}

int send_data_to_server(void* data, uint32_t size) {
    if (!keep_working) {
        return -1;
    }
    if (size > BUFLEN)
        return -1;
    uint32_t current_offset = 0;
    int size_send = 0;
    char buf_msg[BUFLEN] = { 0 };
    char* temp_d = reinterpret_cast<char*>(data);
    memcpy(buf_msg, temp_d, size);
    if ((size_send = sendto(s, buf_msg, BUFLEN, 0, (struct sockaddr*)&si_other, slen)) == SOCKET_ERROR)
    {
        return -1;
    }
    return size_send;
}
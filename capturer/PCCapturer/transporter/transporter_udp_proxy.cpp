#include "transporter_udp_proxy.h"
#include "../result_writer.h"

void from_json(const json& j, ProxyConfig& p) {
	j.at("selfPort").get_to(p.self_port);
	j.at("selfAddr").get_to(p.self_addr);
	j.at("srvPort").get_to(p.srv_port);
	j.at("srvAddr").get_to(p.srv_addr);
	j.at("srvPath").get_to(p.srv_path);
	j.at("isIndi").get_to(p.is_indi);
}

// 172.22.107.250
//#define SERVER "193.190.127.246"	//ip address of udp server NO LONGER USED
#define BUFLEN 1500	//Max length of buffer
#define PORT 8001
void TransporterUDPProxy::Init(const std::string& config_path) {
	std::cout << config_path << std::endl;
	std::ifstream f(config_path);
	json data = json::parse(f);
	std::cout << "test" << std::endl;
	
	config = data.template get<ProxyConfig>();
	std::cout << config.srv_path << std::endl;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("WSA ERROR");
		exit(EXIT_FAILURE);
	}

	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
	{
		printf("SOCKET ERROR");
		WSACleanup();
		exit(EXIT_FAILURE);
	}
	si_self.sin_family = AF_INET;
	si_self.sin_port = htons(config.self_port);
	si_self.sin_addr.s_addr = htonl(INADDR_ANY);
	if (::bind(s, (struct sockaddr*)&si_self, sizeof(si_self)) < 0) {
		exit(EXIT_FAILURE);
	}
	ULONG buf_size = 524288000;
	setsockopt(s, SOL_SOCKET, SO_RCVBUF, (char*)&buf_size, sizeof(ULONG));
	std::cout << config.srv_path << std::endl;
	StartPeerProcess(config.srv_path, 
		config.self_addr + ":" + std::to_string(config.self_port), 
		config.srv_addr + ":" + std::to_string(config.srv_port),
		true, true, "Test Peer", config.is_indi
	);
}

void TransporterUDPProxy::StartPeerProcess(
	const std::string& peer_executable_path,
	const std::string& capturer_addr,
	const std::string& svr_addr,
	bool peer_in_window,
	bool peer_dont_close,
	const std::string& name,
	bool is_indi
) {
	std::ostringstream args;
	args << "-p -cap " << capturer_addr
		<< " -srv " << svr_addr;
	if (is_indi) {
		args << " -i";
	}
	std::string command = peer_executable_path + " " + args.str();

	if (peer_in_window && peer_dont_close) {
		command = "start CMD.EXE /K " + command;
	}

	std::cout << name << ": Start peer: " << command << std::endl;

	int result = std::system(command.c_str());
	if (result != 0) {
		std::cerr << name << ": Cannot start peer" << std::endl;
	}
}

// Connection setup performed by sending / receiving a simple hello message
void TransporterUDPProxy::SetupConnection() {
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(config.srv_port);
	inet_pton(AF_INET, config.srv_addr.c_str(), &si_other.sin_addr.S_un.S_addr);

	pollfds[0].fd = s;
	pollfds[0].events = POLLIN;

	InputPacket p = PollNextPacket(1500);
	std::cout << "Start polling" << std::endl;
	bool is_ready = false;
	while (!is_ready) {
		while (p.status != 0) {
			p = PollNextPacket(1500);
		}
		uint32_t p_type;
		std::memcpy(&p_type, p.data, sizeof(p_type));
		if (p_type == ReadyPacketType) {
			is_ready = true;
		}
	}
	
	std::cout << "End polling" << std::endl;
	char buf_msg[1300];
	struct BitrateRequestHeader p_bit {  };
	memcpy(buf_msg + 4, &p_bit, sizeof(p_bit));
	SendPacket(buf_msg, sizeof(p_bit), ReadyPacketType);
	isClientConnected = true;
}

void TransporterUDPProxy::SendEncodedData(size_t size, const char* encoded_pc, bool indi, uint64_t clientID) {
	std::unique_lock lk(m);
	int frame_counter = 0;
	auto it = frame_counters.find(clientID);
	if (it != frame_counters.end()) {
		frame_counter = it->second;
	}  
	uint32_t current_offset = 0;
	int remaining = size;
	size_t data_size = (1300 - sizeof(VideoPacketHeader));
	size_t next_size = data_size;
	int size_send = 0;
	int size_full = 0;
	// Output every 100 frames
	if (frame_counter % 100 == 0)
	{
		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()
		);
		std::cout << "[CLIENT " << clientID << "] Frame Complete " << frame_counter << " TS = " << ms.count() << std::endl;
		// Remove newly added frame to make sure that memory doesn't get full
	}

	// Send in small packets instead of complete frame
	// Prevent IP layer fragmentation
	/*if (indi) {
		packetType = clientID;
	}*/
	uint32_t bufn_h = 1300 - sizeof(VideoPacketHeader) - 4;
	while (remaining > 0) {
		char buf_msg[1300];
		uint32_t next_size = 0;
		if (remaining >= bufn_h) {
			next_size = bufn_h;
		}
		else {
			next_size = remaining;
		}
		struct VideoPacketHeader p { clientID, frame_counter, size, current_offset, next_size };
		memcpy(buf_msg+4, &p, sizeof(p));
		memcpy(buf_msg + sizeof(p)+4, encoded_pc + current_offset, next_size);
		
		SendPacket(buf_msg, next_size + sizeof(p), FramePacketType);
		size_full += size_send;
		current_offset += next_size;
		remaining -= next_size;
	}
	//ResultWriter::addSendTimestamp(frame_counter, size_full);
	//ResultWriter::setFrameReadyToSave(frame_counter);
	frame_counter++;
	frame_counters[clientID] = frame_counter;
	return;
}

InputPacket TransporterUDPProxy::PollNextPacket(uint32_t buf_size)
{	
	if (buf_size > input_buffer.capacity()) {
		input_buffer.reserve(buf_size);
	}
	int result = WSAPoll(pollfds, 1, 5000);
	if (result == SOCKET_ERROR) {
		// Error handling
		return InputPacket{ 1 };
	}
	else if (result == 0) {
		// Timeout
		return InputPacket{ 2 };
	}
	size_t packet_len;
	if ((packet_len = recvfrom(s, input_buffer.data(), buf_size, 0, (struct sockaddr*)&si_other, &slen)) == SOCKET_ERROR)
	{
		return InputPacket{3};
	}
	return InputPacket{ 0, packet_len, input_buffer.data() };
}

std::map<uint64_t, uint32_t> TransporterUDPProxy::GetClientBitrates() {
	std::unique_lock lk(m);
	std::map<uint64_t, uint32_t> client_bitrates;
	size_t data_size = (1300 - sizeof(BitrateRequestHeader));
	int size_send = 0;
	char buf_msg[1300];
	struct BitrateRequestHeader p_bit {  };
	memcpy(buf_msg+4, &p_bit, sizeof(p_bit));
	SendPacket(buf_msg, sizeof(p_bit), ControlPacketType);
	bool is_ready = true;
	while (is_ready) {
		InputPacket p = PollNextPacket(1500);
		uint32_t p_type;
		std::memcpy(&p_type, p.data, sizeof(p_type));
		//std::cout << "ptype" << p_type << std::endl;
		if (p_type == ControlPacketType) {
			uint32_t n_clients;;
			// P_TYPE 4
			// N_CLIENTS 4
			// CLIENT_ID +4
			// CLIENT_BW +4
			std::memcpy(&n_clients, p.data + 4, sizeof(n_clients));
			//std::cout << n_clients << std::endl;
			size_t offset = 4 + sizeof(n_clients);
			for (uint32_t i = 0; i < n_clients; ++i) {
				uint32_t key;
				uint32_t bitrate;
				// Read key
				std::memcpy(&key, p.data + offset, sizeof(key));
				offset += sizeof(key);
				std::memcpy(&bitrate, p.data + offset, sizeof(bitrate));
				offset += sizeof(bitrate);
				// Insert into the map
				//std::cout << key << " " << bitrate << std::endl;
				client_bitrates[key] = bitrate;
				//std::cout << key << " " << bitrate << std::endl;
			}
			is_ready = false;
		}
	}
	
	
	return client_bitrates;
}

int TransporterUDPProxy::SendPacket(char* data, uint32_t size, uint32_t _packet_type) {
	// Required parameters
	uint32_t packet_type = _packet_type;
	int size_sent = 0;

	// Insert all data into a buffer
	memcpy(data, &packet_type, sizeof(packet_type));
	// Send the message to the Golang peer
	if ((size_sent = sendto(s, data, size + 4, 0, (struct sockaddr*)&si_other, slen)) == SOCKET_ERROR) {
		return -1;
	}

	// Return the amount of bytes sent
	return size_sent;
}

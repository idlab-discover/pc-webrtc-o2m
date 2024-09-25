# WebRTC based Point Cloud Streaming Client
This repository contains the receiving WebRTC client of the real-time point cloud streaming pipeline. The project structure is fairly simple, at the start it uses WebSockets to setup the WebRTC connection with the server and afterwards the client receives packetized frames from the server which are optionally forwarded to a proxy application using a UDP socket. NACK based retranmssions are also enabled so frame reception is guaranteed as long as packet loss doesn't go too high.

# Building
Building the project requires the use of Golang. To ensure comptability Golang version 1.21+ should be used. However, older version might also work but have not yet been tested. The project itself has been tested on both Windows and Ubuntu 20.04.

# Dependencies (will be installed by Golang)
* [Pion (WebRTC)](https://github.com/pion/webrtc)
* [Gorilla (websockets)](https://github.com/gorilla/websocket)

# Usage
The client should be started after the server is fully setup to ensure that the signaling server can be contacted. Following command line parameters can be used to change the behaviour of the application:

| **Parameter** | **Name**     | **Description**                                                       | **Example**    |
|---------------|------------- |-----------------------------------------------------------------------|----------------|
| -v            | IP Filter    | Forces the client to communicate using the interface with this IP     | 192.168.10.1   |
| -w            | WS Server ip | IP Address of the websocket server used for the signalling proces     | localhost:5678 |
| -p            | Proxy Port   | If enabled the client will forward frames to a separate application   | :8001          |
| -m            | Result Path  | The path to which metrics are saved (folder + file without extension) | results/exp_1  |

# Roadmap

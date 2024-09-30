# WebRTC based Point Cloud Streaming Server
This repository contains the code to build the WebRTC server application, which makes use of the [pion](https://github.com/pion/webrtc) as the backbone implementation of the whole WebRTC stack. Signalling (i.e., exchange of the SDP and ICE candidates) is performed using a Websocket server that is included in the WebRTC server. This Websocket server is also used to transmit the current position and field of view of the client to the server.

The server contains an bitrate allocation algorithm which uses the bandwidth estimated by GCC to decide which quality of each user has to be send to the client. Additionally, it uses position and field of view of the client to improve overall quality by giving more quality to those users that are closer to the client.

For a more detailed explanation on how this algorithm works, we refer to our recent publication [1].

[1] M. De Fré, J. van der Hooft, T. Wauters, and F De Turck. "Scalable MDC-Based Volumetric Video Delivery for Real-Time One-to-Many WebRTC Conferencing", Proceedings of the 15th ACM Multimedia Systems Conference, 2024 (available [here](https://backoffice.biblio.ugent.be/download/01HW2J66EZD49XQD2P94JBXHKR/01HW2J8F937QNC36XHZEBRHE8K))

# Usage
The  server application will automically be started by the capturing application, which uses a config file to configure the following parameters:

| **Parameter** | **Name**           | **Description**                                                          | **Example**    |
|---------------|--------------------|--------------------------------------------------------------------------|----------------|
| -cap            | Capturer Address          | What address + port is used by the capturer        | 127.0.0.1:7000   |
| -srv            | Server Address         | What address + port should the server use to communicate with the capturer | 127.0.0.1:7001          |
| -i            | Is individual coding used  | This will send the bitrates from the server to the capturer to allow for individual encoding | false

## Building
Simply use: `go build -o server.exe .` to build the application.


## Dependencies
You do not need to worry about any dependencies as Golang will automatically download them for you.
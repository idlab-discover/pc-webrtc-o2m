# WebRTC based Point Cloud Streaming Client
This repository contains the code to build the WebRTC client application, which makes use of the [pion](https://github.com/pion/webrtc) as the backbone implementation of the whole WebRTC stack. Signalling (i.e., exchange of the SDP and ICE candidates) is performed using a Websocket server that is included in the WebRTC server. This Websocket server is also used to transmit the current position and field of view of the client to the server.


# Usage
The client application will automically be started by the Unity application, which uses a config file to configure the following parameters:

| **Parameter** | **Name**     | **Description**                                                       | **Example**    |
|---------------|------------- |-----------------------------------------------------------------------|----------------|
| -p            | Unity Address    | Which address is used by Unity to communicate with WebRTC     | 127.0.0.1:8000   |
| -clt            | Proxy WebRTC Address  | Which address is used the WebRTC peer to communicate with Unity| 127.0.0.1:8001 |
| -srv            | WebRTC Server Address   | The address of the WebRTC (capturer) server   | 192.168.10.1:8001          |


## Building
Simply use: `go build -o client.exe .` to build the application.

## Dependencies
You do not need to worry about any dependencies as Golang will automatically download them for you.
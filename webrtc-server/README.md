# WebRTC based Point Cloud Streaming Server
This repository contains the sending WebRTC server of the real-time point cloud streaming pipeline. The project contains a built in signalling server which is used to exchange SDP messages with the clients. The server is able to transmit data from either a directory within the project or by using point clouds captured by the [PC capturing application](https://github.com/MatthiasDeFre/pc-capturer ). NACK based retranmssions are also enabled so frame reception is guaranteed as long as packet loss doesn't go too high. Bandwidth is estimated using [Google Congestion Control (GCC)](https://datatracker.ietf.org/doc/html/draft-ietf-rmcat-gcc-02) and the quality of the content is adapted using this estimation.

# Building
Building the project requires the use of Golang. To ensure comptability Golang version 1.21+ should be used. However, older version might also work but have not yet been tested. The project itself has been tested on both Windows and Ubuntu 20.04.


# Dependencies (will be installed by Golang)
* [Pion (WebRTC)](https://github.com/pion/webrtc)
* [Gorilla (websockets)](https://github.com/gorilla/websocket)

# Usage
Following command line parameters can be used to change the behaviour of the application:

| **Parameter** | **Name**           | **Description**                                                          | **Example**    |
|---------------|--------------------|--------------------------------------------------------------------------|----------------|
| -v            | IP Filter          | Forces the server to communicate using the interface with this IP        | 192.168.10.1   |
| -p            | Proxy Port         | If enabled the server will receive frames from a UDP socket on this port | :8001          |
| -d            | Content Directory  | When not using the proxy port, a folder with content can be used instead | content_madfr  |
| -f            | Content Frame Rate | When not using the proxy port, the FPS at which the content is server    | 30             |
| -s            | Signaling IP       | IP on which the signaling server will be created                         | 127.0.0.1:5678 |
| -m            | Result Path        | The path to which metrics are saved (folder + file without extension)    | results/exp_1  |

To test the application you can use the following test content: [900 frame test sequence](https://drive.google.com/file/d/1yYDy3GVNkUxuNm5Qfs_-1BTZ6MbLrm7Y/view?usp=sharing)

# Roadmap
* Add support for many-to-many communication
* Add support for tile-based / part of object adaptation

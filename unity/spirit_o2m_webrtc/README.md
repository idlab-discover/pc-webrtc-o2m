# Point Cloud Streaming Renderer
The repository is the final stage of the real-time point cloud based streaming pipeline ([Complete pipeline](https://github.com/MatthiasDeFre/webrtc-pc-streaming)). It focuses on the rendering and decoding of point clouds, displaying them in the Unity application and subsequently on a connected HMD. This application makes use of a UDP socket to receive the encoded point clouds from the WebRTC client. It makes use of a C++ library which can be found here: https://github.com/MatthiasDeFre/pc-streaming-unity-plugin

## Building
Currently the project can only be build on Windows using Unity together with Visual Studio. This limitation exist due to very limited VR support for other platforms.

## Dependencies
This project requires several dependancies which are included in the Unity project, no manual installation is required.

* [Draco (point cloud decoder)](https://github.com/atteneder/DracoUnity)
* [Pcx (point cloud renderer)](https://github.com/keijiro/Pcx)

## Usage
* The application uses a config file to automatically start the WebRTC application (you will probably only have to change the `srvAddr`).
* The config file should be placed in a config directory situated at: [Application.dataPath](https://docs.unity3d.com/ScriptReference/Application-dataPath.html).
* The peer application should be placed in a peer directory situated at: [Application.dataPath](https://docs.unity3d.com/ScriptReference/Application-dataPath.html).
* You can find an example of both in the `Àssets` directory.
* An HMD can be used, the application was tested with a Meta Quest 2 together with the link app. 


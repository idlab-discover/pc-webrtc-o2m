# Point Cloud Streaming Renderer
The repository contains the Unity project that serves as the client in the one-to-many streaming pipeline. This application is responsible for receiving, decoding and transmitting the camera position and field of view to the SFU. 

As Unity does not have solid support for WebRTC, a seperate Golang based application is used. This application allows us to make use of advanced WebRTC features, such as GCC based bandwidth estimation and NACK retransmissions. Unity communicates with this external application (which is automatically started by Unity) using sockets.

## Project structure

- Scenes: contains the scenes of the project (you will mostly need to use `MainScene`)
- Scripts: contains the C# scripts used by the application
- Prefabs: contains the Unity `GameObject` prefabs for sending and receiving
- Plugins: contains all the Dlls used by the application

## Building
Building the application is very simple, and is the same as building a normal Unity application. 

However, after building the application, you will have to manually copy the `config` and `peer` directories from the `Assets` directory to the [Unity application datapath](https://docs.unity3d.com/ScriptReference/Application-dataPath.html) (In Windows this is: `spirit_unity_Data`).

## Usage
The arrow keys can be used to move the camera when not using any headset. If you are using a headset, make sure that headset is fully connected to your pc (e.g., for Meta Quest, make sure you are fully linked before starting the application).

## Usage
* The application uses a config file to automatically start the WebRTC application (you will probably only have to change the `srvAddr`).
* The config file should be placed in a config directory situated at: [Application.dataPath](https://docs.unity3d.com/ScriptReference/Application-dataPath.html).
* The peer application should be placed in a peer directory situated at: [Application.dataPath](https://docs.unity3d.com/ScriptReference/Application-dataPath.html).
* You can find an example of both in the `Àssets` directory.


## Dependencies
All dependencies will be automatically downloaded when opening the project.

## Supported HMDs
In general every OpenXR compatible headset will work. However, below is a list of all headsets that have been tested and verified:
#### Tested
- Meta Quest 2

# Point Cloud Capturing / Preprocessing
The repository is the first stage of the real-time point cloud based streaming pipeline ([Complete pipeline](https://github.com/MatthiasDeFre/webrtc-pc-streaming)). It focuses on the capturing, preprocessing and encoding of point clouds and preparing them for low-latency transport between a single server and multiple clients. This application makes use of a UDP socket to transport the encoded point clouds to a seperate application responsible for the transmission between the server and clients. Configuration of the socket IP and port can be found at: [Usage](https://github.com/MatthiasDeFre/pc-capturer ) 

## Project Structure
The project is build in a modular way allowing you to provide your own implementations of each pipeline stage. An interface is provided for the capturing, sampling and encoding stages.  For capturing we currently use the Intel Realsense SDK 2.0. However, it is completely viable to implement different point cloud capturing solutions as long as you write your own code which converts the captured point cloud into a PCL compatible format for the other stages of the pipeline.

## Building
Currently the project can only be build on Windows by using the supplied [Visual Studio](https://visualstudio.microsoft.com/) solution. When building the project for testing you should make sure that you selected the release candidate, this is due to certain dependencies exhibiting suboptimal behaviour when using the debug build which can hinder the real-tine effeciency of the application. 

## Dependencies
This project requires several dependancies for the capturing and preprocessing of the point clouds. You can either build and install these dependencies yourself (and make sure the Visual Studio project is able to find them) or if you have vcpkg all required dependencies will be automatically installed and build (currently this application uses PCL which takes significant time to build with vcpkg).

<ul>
  <li> 
    <a href="https://www.intelrealsense.com/sdk-2/">Intel Realsense SDK 2.0</a>
    <a href="https://github.com/IntelRealSense/librealsense">(Code)</a>
  </li>
  <li> 
    <a href="https://pointclouds.org/">Point Cloud Library (PCL)</a>
    <a href="https://github.com/PointCloudLibrary/pcl">(Code)</a>
  </li>
  <li> 
    <a href="https://google.github.io/draco/">Draco</a>
    <a href="https://github.com/google/draco">(Code)</a>
  </li>
</ul>

## Usage
Following command line parameters can be used to change the behaviour of the application. If you provide a config file the Golang peer will automatical start as well. You can find an example of a config file in the `config_example` directory.

| **Parameter** 	| **Name**          	| **Description**                                                      	| **Example**     	|
|---------------	|-------------------	|----------------------------------------------------------------------	|-----------------	|
| -r            	| Result File       	| Location and file name where metrics will be saved                   	| results_100.csv 	|
| -p            	| Proxy Config       	| Config file used to automatically start the Golang peer              	| config.json      	|
| -i            	| Use Camera        	| Use a Realsense camera as input for the pipeline                     	| n.a.            	|
| -d            	| Content Directory 	| When not using a camera .ply files in this location are used instead 	| frames          	|
| -l            	| Encoding strategy 	| Which encoding strategy should be used (e.g., `layer`, `indi`, `fixed_pc`) 	| layer          	|

## Encoding Strategies
The application contains several encoding strategies (set by using the `-l` parameter`) that you can use to encode the point cloud.

### Layer
This uses an MDC-based approach which splits the point cloud into three base descriptions containing 15%, 25% and 60% of the points in the original point cloud. The server will then decide which of these descriptions will be forwarded to the user based on their available bandwidth and position. Each of these base descriptions contains distinct points, because of this a user can receive multiple base descriptions and combine them into a higher quality version.

### Indi
This method uses the available bitrate of a user to calculate a respective sampling rate. This results in each user having the best possible quality. However, because now encoding has to be done for each user seperately, the maximum number of users will be selected based on the hardware of the capturer.

### Fixed
This method is similar to the `layer` approach, but will first limit the number of points in the point cloud to `125.000`.
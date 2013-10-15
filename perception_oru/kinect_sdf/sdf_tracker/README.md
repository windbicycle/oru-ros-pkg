#README

The standalone version of CMakeLists.txt will allow compilation outside the rosbuild environment.
This is not very well tested at the moment. 

##Dependencies:

* eigen 3
* vtk 5.8
* boost-filesystem 1.53 (maybe 1.49 is ok too)
* OpenCV 2.x
* cmake 
* OpenNI2

in Ubuntu these can be obtained by executing:
```bash
sudo apt-get install libeigen3-dev libboost-filesystem1.53-dev libopencv-dev libvtk5-dev cmake
```
for OpenNI2, in a directory of your own choosing:
```bash
git clone https://github.com/OpenNI/OpenNI2.git
cd OpenNI2
make
```
Environment variables OPENNI2_REDIST and OPENNI2_INCLUDE must be set to point to the location of libOpenNI2.so and OpenNI.h respectively. 
A suggestion is to add these to your .bashrc file.
```bash
export OPENNI2_REDIST=/home/username/where_I_keep_code_n_stuff/OpenNI2/Bin/x64-Release/
export OPENNI2_INCLUDE=/home/username/where_I_keep_code_n_stuff/OpenNI2/Include/
```
Alternatively, precompiled binaries and more instructions can be found at www.openni.org 

Optionally it is recommended to install paraview for viewing the reconstructed models and associated weights in a graphic environment
```bash
sudo apt-get install paraview 
```
It may have some conflicts with certain ROS-related packages.

##Building

Rename CMakeLists_STANDALONE.txt to CMakeLists.txt

Building the project **should** then be as simple as executing the following commands (from the project directory)
```bash
mkdir build
cd build
cmake ..
make
```

The sdf_tracker_app is a quite minimal example of the SDF_tracker in use and uses OpenNI2 to capture depth images.

For "normal" use within the ROS framework, documentation can be found at http://ros.org/wiki/sdf_tracker

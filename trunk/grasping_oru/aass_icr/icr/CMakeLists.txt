cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

SET(debug_mode $ENV{DEBUG})
IF(DEFINED debug_mode)
  SET( CMAKE_CXX_FLAGS "-D DEBUG")
ENDIF(DEFINED debug_mode)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_executable(icr_server src/icr_server.cpp src/icr_server_node.cpp)
rosbuild_add_executable(model_server src/model_server.cpp src/model_server_node.cpp src/pose_broadcaster.cpp)
rosbuild_add_executable(grasp_server src/grasp_server.cpp src/grasp_server_node.cpp src/phalange.cpp)
#rosbuild_add_executable(icr_client src/icr_client.cpp)
#rosbuild_add_executable(finger_tip_publisher_node src/finger_tip_publisher_node.cpp)
#rosbuild_add_executable(icr_pointcloud_test src/icr_pointcloud_test.cpp)


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

set(LIBRARY_SOURCES 
	src/RobotKinematics.cc
	src/NDTWavefrontPlanner.cc
    )
include_directories(${PROJECT_SOURCE_DIR}/../ndt_map/include)
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
add_definitions(-DDEBUG)
link_directories (${PROJECT_SOURCE_DIR}/../ndt_map/lib)
rosbuild_add_library(${PROJECT_NAME} ${LIBRARY_SOURCES})
target_link_libraries(${PROJECT_NAME} ndt_map)

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(ndtWavefrontTest test/ndtWavefrontTest.cc)
target_link_libraries(ndtWavefrontTest ndt_map ${PROJECT_NAME} config++ pointcloud_vrml)

rosbuild_add_executable(extractDrivable test/ExtractDrivable.cc)
target_link_libraries(extractDrivable ndt_map ${PROJECT_NAME} config++ pointcloud_vrml)

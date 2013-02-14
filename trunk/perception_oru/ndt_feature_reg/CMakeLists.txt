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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/NDTFrame.cc src/NDTFrameProc.cc src/NDTFrameViewer.cc )
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#find_package(Boost REQUIRED)
#include_directories(${Boost_INCLUDE_DIRS})

#rosbuild_add_executable(frame_matching_test test/frame_matching_test.cc)
#target_link_libraries(frame_matching_test ${PROJECT_NAME})

#TSV::
#rosbuild_add_executable(openmp_test test/openmp_test.cpp)
#rosbuild_add_openmp_flags(openmp_test)
#target_link_libraries(openmp_test)

#rosbuild_add_executable(ndt_feature_viewer src/ndt_feature_viewer.cc)
#rosbuild_link_boost(ndt_feature_viewer program_options thread)
#target_link_libraries(ndt_feature_viewer ${PROJECT_NAME})

#rosbuild_add_executable(simple_cloud_viewer test/simple_cloud_viewer.cc)
#target_link_libraries(simple_cloud_viewer)

#rosbuild_add_executable(affine_transform_test test/affine_transform_test.cc)
#target_link_libraries(affine_transform_test)

rosbuild_add_executable(ndt_feature_eval src/ndt_feature_eval.cc)
rosbuild_link_boost(ndt_feature_eval program_options thread)

rosbuild_add_executable(ndt_feature_reg_node src/ndt_feature_reg_node.cc)
rosbuild_link_boost(ndt_feature_reg_node signals program_options thread)

rosbuild_add_executable(exp_ndt_feature_reg_node src/exp_ndt_feature_reg_node.cc)
rosbuild_link_boost(exp_ndt_feature_reg_node signals program_options thread)


rosbuild_get_stack_version(ROS_VERSION ros)

STRING(REGEX REPLACE "^([0-9]+)\\.[0-9]+\\.[0-9]+" "\\1" ros_major 
"${ROS_VERSION}")
STRING(REGEX REPLACE "^[0-9]+\\.([0-9])+\\.[0-9]+" "\\1" ros_minor 
"${ROS_VERSION}")
STRING(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)" "\\1" ros_patch 
"${ROS_VERSION}")

if(ros_major MATCHES 1 AND ros_minor GREATER 7)
MESSAGE(STATUS "ROS version: fuerte")
	include_directories(/usr/include/vtk-5.8)
#target_link_libraries(ndt_feature_eval ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
#	target_link_libraries(ndt_feature_reg_node ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
#	target_link_libraries(ndt_feature_viewer ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
endif()

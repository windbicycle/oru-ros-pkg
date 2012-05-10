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

#set( LIBRARY_SOURCES
#        src/OctTree.cc	
#	src/Cell.cc 
#	src/NDTCell.cc
#	src/NDTMap.cc
#	src/NDTHistogram.cc
#	src/AdaptiveOctTree.cc
#	src/Pose.cc
#	src/LazzyGrid.cc
#	src/CellVector.cc
#	)
#make a library with the core functionalities (depends on eigen, tf and pcl)
#rosbuild_add_library(${PROJECT_NAME} ${LIBRARY_SOURCES})
#target_link_libraries(${PROJECT_NAME} another_library)

#make a node that listens to point clouds and makes ndts

#rosbuild_add_executable(oct_builder test/octTreeBuilder.cc)
rosbuild_add_executable(ndt_builder test/ndtMapBuilder.cc)
rosbuild_add_executable(histTest test/test_ndt_histogram.cc)

#rosbuild_add_executable(likelihood_test test/ndtLikelihoodTester.cc)
#target_link_libraries(likelihood_test ${PROJECT_NAME}  pointcloud_vrml)

#rosbuild_add_executable(ltest test/likelihoodSingleScan.cc)
#target_link_libraries(ltest ${PROJECT_NAME}  pointcloud_vrml)

#rosbuild_add_executable(batchTestHist test/batchTesterHistogramFPFH.cc)
#target_link_libraries(batchTestHist ${PROJECT_NAME} pointcloud_vrml)
#common commands for building c++ executables and libraries
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

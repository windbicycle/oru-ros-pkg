cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)

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
#rosbuild_add_library(${PROJECT_NAME} ${LIBRARY_SOURCES})
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_add_executable(ndt_registration_test test/ndt_registration_test.cc)
#rosbuild_add_openmp_flags(ndt_registration_test)

rosbuild_add_executable(simple_test test/simple.cc)
rosbuild_add_openmp_flags(simple_test)

rosbuild_add_executable(from_jff test/from_jff.cc)
rosbuild_add_openmp_flags(from_jff)

#target_link_libraries(ndt_registration_test ndt_map pointcloud_vrml)

#rosbuild_add_executable(ndt_feature_test test/ndt_feature_test.cc)
#rosbuild_link_boost(ndt_feature_test program_options)

#target_link_libraries(ndt_feature_test ${PROJECT_NAME} ndt_map pointcloud_vrml)

#rosbuild_add_executable(feature_registration test/feature_registration.cc)
#target_link_libraries(feature_registration ndt_map pointcloud_vrml )

#rosbuild_add_executable(ndt_reconstruct test/reconstruct.cc)
#target_link_libraries(ndt_reconstruct ${PROJECT_NAME} ndt_map pointcloud_vrml )

#rosbuild_add_executable(ndt_matcher_node test/ndt_matcher_node.cc)
#target_link_libraries(ndt_matcher_node ${PROJECT_NAME} ndt_map pointcloud_vrml )

#rosbuild_add_executable(batchTestRangerAccuracy test/batchTestRangerAccuracy.cc)
#target_link_libraries(batchTestRangerAccuracy ${PROJECT_NAME} ndt_map  pointcloud_vrml)

#rosbuild_add_executable(narf_keypoint_extraction test/narf_extraction.cc)
#target_link_libraries(narf_keypoint_extraction ${PROJECT_NAME} ndt_map pointcloud_vrml )

#rosbuild_add_executable(narf_registration test/narf_registration.cc)
#target_link_libraries(narf_registration ${PROJECT_NAME} ndt_map pointcloud_vrml )

#rosbuild_add_executable(ndt_histogram_registration test/ndt_histogram_registration.cc)
#target_link_libraries(ndt_histogram_registration ${PROJECT_NAME} ndt_map pointcloud_vrml )

#rosbuild_add_executable(test_covariance test/test_covariance.cc)
#target_link_libraries(test_covariance ${PROJECT_NAME} ndt_map pointcloud_vrml )

#target_link_libraries(test_ndt_mapper ${PROJECT_NAME} ndt_map pointcloud_vrml )


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
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")


#To use pangolin
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(FREEGLUT REQUIRED)
INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIR})
## Find required dependencies
FIND_PACKAGE(OpenGL REQUIRED QUIET)
FIND_PACKAGE(GLEW REQUIRED)
## Apply required dependencies settings
LIST(APPEND LINK_LIBS "${OPENGL_LIBRARIES};${GLEW_LIBRARY};${Pangolin_LIBRARY};${FREEGLUT_LIBRARY};${OPENGL_LIBRARIES}" )
LIST(APPEND LIB_DIRS  "${OPENGL_LIBRARY_DIR};${GLEW_LIBRARY_DIR};${Pangolin_LIBRARY_DIRS}" )
LINK_DIRECTORIES(${LIB_DIRS})
LINK_LIBRARIES(${LINK_LIBS})
MESSAGE(STATUS "Found Pangolin: ${Pangolin_LIBRARY}")
MESSAGE(STATUS "link dirs: ${LIB_DIRS}")
MESSAGE(STATUS "link libs: ${LINK_LIBS}")

#### need to add something so rosmake also build pangolin library ####
#ADD_SUBDIRECTORY(Pangolin/build)
######################################################################

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(jan_test src/main.cpp)
rosbuild_add_executable(ndt_visualiser src/fromFile.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(ndt_visualiser filesystem system thread)

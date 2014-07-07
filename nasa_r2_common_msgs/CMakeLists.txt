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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_find_ros_package(nasa_cmake)
list(APPEND CMAKE_MODULE_PATH ${nasa_cmake_PACKAGE_PATH})
find_package(Log4cpp REQUIRED)
include_directories(${Log4cpp_INCLUDE_DIR})
#find_package(ProtocolBuffers REQUIRED)
#include_directories(${ProtocolBuffers_INCLUDE_DIR})

# load r2 cmake files
rosbuild_find_ros_package(nasa_cmake)
list(APPEND CMAKE_MODULE_PATH ${nasa_cmake_PACKAGE_PATH})

#find_package(ProtocolBuffers REQUIRED)
#set(PROTO_FILES ${PROJECT_SOURCE_DIR}/proto/ProbeMsgs.proto
#                ${PROJECT_SOURCE_DIR}/proto/RTCRemoteMsgs.proto)
#run_protogen("${PROTO_FILES}")

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/RosMsgPrinter.cpp)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

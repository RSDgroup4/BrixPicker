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
rosbuild_gensrv()

rosbuild_add_boost_directories()

### Serial driver ###
rosbuild_add_executable(serial_string src/serial_port/serial_string/serial_string.cpp)
rosbuild_add_executable(serial_string src/serial_port/serial_string/serialInterface.cpp)
rosbuild_link_boost(serial_string system thread)

rosbuild_add_executable(serial_binary src/serial_port/serial_binary/serial_binary.cpp)
rosbuild_add_executable(serial_binary src/serial_port/serial_binary/serial_binary_interface.cpp)
rosbuild_link_boost(serial_binary system thread)
### end of Serial driver ###

#### rx60 driver ###
set(SrcFiles
  src/rx60_controller/RX60Driver.cpp
  src/rx60_controller/RX60TCPSocket.cpp)

#link_directories( )

set(LibraryList
  pthread
  curses)

# The shared library to build:
rosbuild_add_library(RX60 SHARED ${SrcFiles})

# Link the standard static libraries with with the shared library:
target_link_libraries(RX60 ${LibraryList})

add_executable(test-RX60 src/rx60_controller/test.cpp)
target_link_libraries(test-RX60 RX60)
rosbuild_link_boost(test-RX60 system thread)

add_executable(rx60_controller src/rx60_controller/rx60_controller_node.cpp
							   src/rx60_controller/RX60Driver.cpp)
target_link_libraries(rx60_controller RX60)
rosbuild_link_boost(rx60_controller system thread)

### end of rx60 driver ###


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

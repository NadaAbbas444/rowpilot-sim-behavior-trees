cmake_minimum_required(VERSION 3.16)
project(rowpilot_vio_bridge CXX)
# Feeds a simulated (or real) D435i to OpenVINS and publishes the estimated odometry.
#   cmake -S vio_bridge -B build/vio -DOPENVINS_DIR=/path/to/open_vins && cmake --build build/vio
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(OPENVINS_DIR "$ENV{HOME}/open_vins" CACHE PATH "OpenVINS source tree, with ov_msckf/build/libov_msckf_lib.so built")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(vio_bridge src/vio_bridge.cpp)
target_include_directories(vio_bridge PRIVATE
  ${OPENVINS_DIR}/ov_core/src ${OPENVINS_DIR}/ov_init/src ${OPENVINS_DIR}/ov_msckf/src
  ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(vio_bridge PRIVATE
  ${OPENVINS_DIR}/ov_msckf/build/libov_msckf_lib.so
  ${OpenCV_LIBS} ${Boost_LIBRARIES} gz-transport13::core gz-msgs10::core)
set_target_properties(vio_bridge PROPERTIES BUILD_RPATH "${OPENVINS_DIR}/ov_msckf/build")

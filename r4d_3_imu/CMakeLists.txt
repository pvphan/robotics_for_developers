cmake_minimum_required(VERSION 2.8.3)
project(r4d_3_imu)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  r4d_common
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r4d_2_tracking
#  CATKIN_DEPENDS r4d_common
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_node src/imu_node.cpp)

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(OPTIONAL_MKL_LIBS mkl_core mkl_tbb_thread mkl_def mkl_intel_lp64 mkl_intel_ilp64)
endif()

target_link_libraries(imu_node
  ${catkin_LIBRARIES}
  ${OPTIONAL_MKL_LIBS}
  tbb
  gtsam
)

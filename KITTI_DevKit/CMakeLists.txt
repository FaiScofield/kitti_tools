cmake_minimum_required(VERSION 2.8)

project(kitti_dev_kit)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3")

#find_package(Sophus REQUIRED)

include_directories(
#    "/usr/include/eigen3/"
#    ${Sophus_INCLUDE_DIRS}
#    ${OpenCV_INCLUDE_DIRS}
#    ${Pangolin_INCLUDE_DIRS}
)

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


#find_package(Eigen3 3.1.0 REQUIRED)
#find_package(Pangolin REQUIRED)
#find_package(PCL 1.7 REQUIRED )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(evaluate_odometry src/evaluate_odometry.cpp src/matrix.cpp)

add_executable(draw_3_results src/draw_3_results.cpp src/matrix.cpp)
add_executable(draw_odometry src/draw_odometry.cpp src/matrix.cpp)
add_executable(draw_ground_truth src/draw_ground_truth.cpp src/matrix.cpp)

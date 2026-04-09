cmake_minimum_required(VERSION 3.8)
project(mindvision_camera)

include_directories(${PROJECT_SOURCE_DIR}/../../include)

add_executable(cam_test main.cpp ../../src/mindvision_camera.cpp)
target_link_libraries(cam_test MVSDK)

cmake_minimum_required(VERSION 3.8)
project(camera)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set compiler flags
add_compile_options(-Wall -Wextra -Wpedantic)
add_compile_options(-g)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
ament_auto_find_build_dependencies()

# 只编译库文件，不包含带main函数的文件
ament_auto_add_library(${PROJECT_NAME} SHARED
    src/camera_node.cpp
    src/camera4calibrate.cpp
    src/mindvision.cpp
    src/inner_shot.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${cv_bridge_INCLUDE_DIRS} include)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} MVSDK ${cv_bridge_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} cv_bridge)

# 注册组件
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN sensor::CameraNode
    EXECUTABLE camera_node
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN sensor::CameraForCalibrate
    EXECUTABLE camera_for_calibrate
)

# 分别创建独立的可执行文件
ament_auto_add_executable(camera_calibration_capture
    src/camera_calibration_capture.cpp
)

target_include_directories(camera_calibration_capture PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_calibration_capture ${OpenCV_LIBS})

ament_auto_add_executable(calibration_reprojection_error
    src/calibration_reprojection_error.cpp
)

target_include_directories(calibration_reprojection_error PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calibration_reprojection_error ${OpenCV_LIBS})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)
  ament_add_gtest(${PROJECT_NAME}-test test/camera_test.cpp)
  target_include_directories(${PROJECT_NAME}-test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}-test ${OpenCV_LIBS} ${PROJECT_NAME})
endif()

ament_auto_package(
    INSTALL_TO_SHARE
    launch
)
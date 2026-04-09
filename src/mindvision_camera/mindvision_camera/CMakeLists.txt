cmake_minimum_required(VERSION 3.8)
project(mindvision_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

# 查找相机 SDK 头和库
find_path(MVSDK_INCLUDE_DIR
  NAMES CameraApi.h
  HINTS /usr/include/mindvision /usr/local/include/mindvision /usr/local/include /usr/include /home/robot-max/Downloads/linuxSDK_V2.1.0.49202602041120/include
)
find_library(MVSDK_LIBRARY
  NAMES MVSDK
  HINTS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

message(STATUS "mvsdk_cpp: Found MVSDK include dir: ${MVSDK_INCLUDE_DIR}")
message(STATUS "mvsdk_cpp: Found MVSDK lib: ${MVSDK_LIBRARY}")

include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include ${MVSDK_INCLUDE_DIR})

add_executable(camera_main src/node_main.cpp src/camera_node.cpp src/mindvision_camera.cpp)
target_link_libraries(camera_main ${MVSDK_LIBRARY})
ament_target_dependencies(camera_main rclcpp OpenCV sensor_msgs)

install(TARGETS
  camera_main
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件夹
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

# 注册组件
add_library(camera_component SHARED src/node_composable.cpp src/camera_node.cpp src/mindvision_camera.cpp)
target_link_libraries(camera_component MVSDK)
ament_target_dependencies(camera_component rclcpp OpenCV sensor_msgs rclcpp_components)
rclcpp_components_register_node(
    camera_component
    PLUGIN "camera::CameraNode"
    EXECUTABLE camera
)
ament_export_targets(export_camera_component)
install(TARGETS camera_component
        EXPORT export_camera_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

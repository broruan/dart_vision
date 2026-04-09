cmake_minimum_required(VERSION 3.8)
project(detector)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 2. 创建库 (不包含 main 函数)
add_library(detector_lib SHARED
  src/detect.cpp
)

target_include_directories(detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS} # 显式包含 OpenCV 头文件
)

# 使用 ament 的简便方式链接 ROS 依赖
ament_target_dependencies(detector_lib
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# 链接 OpenCV
target_link_libraries(detector_lib
  ${OpenCV_LIBRARIES}
)

# 3. 创建可执行文件 (调用上面的库)
# 假设你有一个 src/detect_node.cpp 里面写了 int main()
add_executable(video_detector_node src/detect_node.cpp)

target_link_libraries(video_detector_node
  detector_lib
)

# 4. 安装
install(TARGETS detector_lib video_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
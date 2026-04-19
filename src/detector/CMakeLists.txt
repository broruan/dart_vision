cmake_minimum_required(VERSION 3.8)
project(detector)

# 配置编译选项
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
find_package(communicate_2025 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 增加接口文件
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/DealImg.msg"
  DEPENDENCIES builtin_interfaces
)

# 2. 编译库 (除了带 main 的)
add_library(detector_lib SHARED
  src/detect.cpp
  src/orbit_calculate.cpp
)

target_include_directories(detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS} # 引入 OpenCV 头文件
)

# 使用 ament 文件方式链接 ROS 依赖
ament_target_dependencies(detector_lib
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  communicate_2025
)

# 链接 OpenCV
target_link_libraries(detector_lib
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 3. 编译可执行文件 (没有带 main 的)
# 这里需要一个 src/detect_node.cpp 里面写上 int main()
add_executable(video_detector_node src/detect_node.cpp)
add_executable(orbti_calculate src/orbit_calculate_node.cpp)

target_link_libraries(video_detector_node
  detector_lib
)

target_link_libraries(orbti_calculate
  detector_lib
)

# 4. 安装
install(TARGETS detector_lib video_detector_node orbti_calculate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

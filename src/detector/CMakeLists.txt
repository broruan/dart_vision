cmake_minimum_required(VERSION 3.8)
project(detector)

# ���ñ���ѡ��
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# 1. ��������
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(communicate_2025 REQUIRED)

# 2. ������ (������ main ����)
add_library(detector_lib SHARED
  src/detect.cpp
)

target_include_directories(detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS} # ��ʽ���� OpenCV ͷ�ļ�
)

# ʹ�� ament �ļ�㷽ʽ���� ROS ����
ament_target_dependencies(detector_lib
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  communicate_2025
)

# ���� OpenCV
target_link_libraries(detector_lib
  ${OpenCV_LIBRARIES}
)

# 3. ������ִ���ļ� (��������Ŀ�)
# ��������һ�� src/detect_node.cpp ����д�� int main()
add_executable(video_detector_node src/detect_node.cpp)

target_link_libraries(video_detector_node
  detector_lib
)

# 4. ��װ
install(TARGETS detector_lib video_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ��װ�����ļ�
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ��װlaunch�ļ�
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
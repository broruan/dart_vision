cmake_minimum_required(VERSION 3.16)
project(communicate_2025)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_BUILD_TYPE "Debug")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SerialInfo.msg"
    "msg/Autoaim.msg"
    "msg/Command.msg"
    DEPENDENCIES
    std_msgs
)

rosidl_get_typesupport_target(
    cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp
)
ament_auto_add_executable(${PROJECT_NAME}_node
    DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}_node
    ${cpp_typesupport_target}
)
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)
ament_auto_package()

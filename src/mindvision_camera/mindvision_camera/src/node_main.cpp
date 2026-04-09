#include "include/mindvision_camera/camera_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera::CameraNode>("camera"));
    rclcpp::shutdown();

    return 0;
}
#include "detector/detect.hpp"


#include <rclcpp/rclcpp.hpp>




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto detector_node = std::make_shared<detector::VideoDetectorNode>("video_detector_node");


    try {
        rclcpp::spin(detector_node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(detector_node->get_logger(), "Exception in main: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
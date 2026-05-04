#include "camera/camera4calibrate.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unistd.h>
#include <stdexcept>

namespace sensor {

CameraForCalibrate::CameraForCalibrate(const rclcpp::NodeOptions& options):
    Node("camera_node", options) {
    RCLCPP_INFO(this->get_logger(), "camera_node start");

    std::string default_camera_config;
    try {
        // default_camera_config = ament_index_cpp::get_package_share_directory("mindvision_camera") + "/Camera/Configs/Front_Camera-Group1.config";
        default_camera_config = ament_index_cpp::get_package_share_directory("mindvision_camera") + "/Camera/Configs/Front_Camera-Group1.config";
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not find mindvision_camera package share: %s", e.what());
    }

    std::string camera_config_path;
    camera_config_path = this->declare_parameter("mindvision_config_path", default_camera_config);
    sn_ = this->declare_parameter("sn", std::string(""));// Ïà»úÐòÁÐºÅ
    publish_fps_ = this->declare_parameter("publish_fps", 60.0);
    if (publish_fps_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid publish_fps %.2f, fallback to 5.0", publish_fps_);
        publish_fps_ = 60.0;
    }

    mindvision_ = std::make_shared<MindVision>(camera_config_path, sn_);
    if (!mindvision_->GetCameraStatus()) {
        RCLCPP_ERROR(this->get_logger(), "mindvision failed: camera not available or initialization failed");
        throw std::runtime_error("mindvision failed");
    }

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_pub",
        rclcpp::SensorDataQoS().keep_last(2)
    );

    frame_ = std::make_shared<cv::Mat>();
    thread_for_publish_ = std::thread(std::bind(&CameraForCalibrate::LoopForPublish, this));
}

CameraForCalibrate::~CameraForCalibrate() {
    if (thread_for_publish_.joinable()) {
        thread_for_publish_.join();
    }
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

void CameraForCalibrate::GetImg() {
    if (!mindvision_->GetFrame(frame_)) {
        failed_count_++;
        RCLCPP_ERROR(this->get_logger(), "mindvision get image failed");
    } else {
        failed_count_ = 0;
    }

    if (failed_count_ > 100) {
        RCLCPP_ERROR(this->get_logger(), "failed too much!");
        rclcpp::shutdown();
    }
}

void CameraForCalibrate::LoopForPublish() {
    rclcpp::Rate rate(publish_fps_);
    while (rclcpp::ok()) {
        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
        image_msg->header.stamp = this->now();
        this->GetImg();

        if (frame_->empty()) {
            rate.sleep();
            continue;
        }

        image_msg->header.frame_id = "camera";
        image_msg->height = frame_->rows;
        image_msg->width = frame_->cols;
        image_msg->encoding = "bgr8";
        image_msg->is_bigendian = 0u;
        image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
        image_msg->data.assign(frame_->datastart, frame_->dataend);

        image_publisher_->publish(std::move(image_msg));
        rate.sleep();
    }
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraForCalibrate)

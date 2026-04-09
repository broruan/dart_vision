#include "camera/camera_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/qos.hpp>
#include <unistd.h>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace sensor {

CameraNode::CameraNode(const rclcpp::NodeOptions& options):
    Node("camera_node", options),
    frame_(std::make_shared<cv::Mat>()),
    debug_auto_aim_mode_sub_(nullptr),
    auto_aim_mode_sub_(nullptr),
    enemy_color_or_rune_flag("armor"),
    mode_(false),
    img_pub_for_rune_(nullptr),
    img_pub_for_armor_(nullptr),
    rune_use_exposure_(4000),
    videoflag(false),
    video_path("/home/robot/1.avi"),
    mindvision_(nullptr),
    capture(),
    thread_for_publish_(),
    thread_for_inner_shot_(),
    inner_shot_flag(false),
    failed_count(0),
    camera_config_path_(""),
    rune_camera_config_path_(""),
    sn_("")
{
    RCLCPP_INFO(this->get_logger(), "camera_node start");

    // 是否使用视频流标志位
    videoflag = this->declare_parameter("videoflag", false);
    video_path = this->declare_parameter("video_path", "/home/robot/1.avi");
    rune_use_exposure_ = this->declare_parameter("rune_exposure", 4000);
    inner_shot_flag = this->declare_parameter("inner_shot_flag", false);

    RCLCPP_INFO(this->get_logger(), "inner_shot flag %d", inner_shot_flag);

    std::string default_camera_config;
    std::string default_rune_config;
    try {
        default_camera_config = ament_index_cpp::get_package_share_directory("auto_aim") + "/config/mindvision.config";
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not find auto_aim package share: %s", e.what());
    }
    try {
        default_rune_config = ament_index_cpp::get_package_share_directory("auto_aim") + "/config/rune_mindvision.config";
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not find auto_aim package share for rune config: %s", e.what());
    }

    camera_config_path_ = this->declare_parameter("mindvision_config_path", default_camera_config);
    rune_camera_config_path_ = this->declare_parameter("rune_mindvision_config_path", default_rune_config);
    sn_ = this->declare_parameter("sn", "");

    mindvision_ = std::make_shared<MindVision>(camera_config_path_, sn_);

    if (videoflag) {
        capture.open(video_path);
        if (!capture.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "video open failed: %s", video_path.c_str());
            throw std::runtime_error("video open failed");
        }
        RCLCPP_INFO(this->get_logger(), "use video: %s", video_path.c_str());
    } else if (!mindvision_->GetCameraStatus()) {
        RCLCPP_ERROR(this->get_logger(), "mindvision failed: camera not available or initialization failed");
        throw std::runtime_error("mindvision failed");
    }

    if (inner_shot_flag) {
        thread_for_inner_shot_ = std::thread(std::bind(&CameraNode::InnerShot, this));
    }

    img_pub_for_rune_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_for_rune",
        rclcpp::SensorDataQoS().keep_last(2)
    );
    img_pub_for_armor_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_for_armor",
        rclcpp::SensorDataQoS().keep_last(2)
    );
    debug_auto_aim_mode_sub_ = this->create_subscription<communicate_2025::msg::Autoaim>("/communicate/debug/autoaim", rclcpp::SensorDataQoS(), std::bind(&CameraNode::AutoAimSwitch, this, std::placeholders::_1));
    auto_aim_mode_sub_ = this->create_subscription<communicate_2025::msg::Autoaim>("/communicate/autoaim", rclcpp::SensorDataQoS(), std::bind(&CameraNode::AutoAimSwitch, this,std::placeholders::_1));
    thread_for_publish_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));
}

void CameraNode::InnerShot() {
    auto inner_shot = std::make_shared<InnerShotNode>();
    RCLCPP_INFO(this->get_logger(), "inner_shot start !.............. ");
    rclcpp::spin(inner_shot);
}

void CameraNode::AutoAimSwitch(const communicate_2025::msg::Autoaim::SharedPtr auto_aim_mode_swicth_msg)
{
    // 模式 0：自瞄 1：符
    if (this->mode_ == auto_aim_mode_swicth_msg->mode) {
        return;
    }
    this->mode_ = auto_aim_mode_swicth_msg->mode == 0 ? false : true;
    if (mode_) {
        // 符曝光
        if (!rune_camera_config_path_.empty()) {
            mindvision_->SetExposureTime(rune_camera_config_path_);
        }
        mindvision_->SetExposureTime(rune_use_exposure_);
        enemy_color_or_rune_flag = "rune";
    } else {
        // 装甲板曝光
        if (!camera_config_path_.empty()) {
            mindvision_->SetExposureTime(camera_config_path_);
        }
        enemy_color_or_rune_flag = "armor";
    }
}

void CameraNode::GetImg() {
    if (videoflag) {
        capture >> *frame_;

        // 循环播放
        if ((*frame_).empty()) {
            RCLCPP_INFO(this->get_logger(), "video end");
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            capture >> *frame_;
        }
    } else {
        if (!mindvision_->GetFrame(frame_)) {
            failed_count++;
            RCLCPP_ERROR(this->get_logger(), "mindvision get image failed");
        } else {
            failed_count = 0;
        }
    }

    if (failed_count > 10) {
        RCLCPP_ERROR(this->get_logger(), "failed too much!");
        rclcpp::shutdown();
    }
}

CameraNode::~CameraNode() {
    if (thread_for_publish_.joinable()) {
        thread_for_publish_.join();
    }
    if (thread_for_inner_shot_.joinable()) {
        thread_for_inner_shot_.join();
    }
    RCLCPP_INFO(this->get_logger(), "camera_node stopped");
}

void CameraNode::LoopForPublish() {
    while (rclcpp::ok()) {
        this->GetImg();
        
        // 检查图像是否有效
        if (frame_->empty()) {
            RCLCPP_WARN(this->get_logger(), "Frame is empty, skipping publish");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = this->enemy_color_or_rune_flag;
        image_msg->height = frame_->rows;
        image_msg->width = frame_->cols;
        image_msg->encoding = "bgr8";
        image_msg->is_bigendian = 0u;
        image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_->step);
        image_msg->data.assign(frame_->datastart, frame_->dataend);
        mode_ ? img_pub_for_rune_->publish(std::move(image_msg)) : img_pub_for_armor_->publish(std::move(image_msg));
    }
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)

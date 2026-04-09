#include "camera/camera_calibration_capture.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sstream>
#include <iomanip>
#include <filesystem>

namespace sensor {

CameraCalibrationCapture::CameraCalibrationCapture(const rclcpp::NodeOptions& options)
    : Node("camera_calibration_capture", options), image_count_(0) {
    
    RCLCPP_INFO(this->get_logger(), "Camera Calibration Capture Node Started");
    
    // 创建保存图像的文件夹
    save_directory_ = declare_parameter("save_directory", std::string("./calibration_images"));
    if (!std::filesystem::exists(save_directory_)) {
        std::filesystem::create_directories(save_directory_);
        RCLCPP_INFO(this->get_logger(), "Created directory: %s", save_directory_.c_str());
    }
    
    // 订阅图像话题
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_pub", 
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&CameraCalibrationCapture::image_callback, this, std::placeholders::_1)
    );
    
    // 创建OpenCV窗口
    cv::namedWindow("Camera Calibration", cv::WINDOW_AUTOSIZE);
    
    RCLCPP_INFO(this->get_logger(), "=== Instructions ===");
    RCLCPP_INFO(this->get_logger(), "Press SPACE to capture image");
    RCLCPP_INFO(this->get_logger(), "Press ESC to exit");
    RCLCPP_INFO(this->get_logger(), "Images will be saved to: %s", save_directory_.c_str());
}

CameraCalibrationCapture::~CameraCalibrationCapture() {
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "Camera Calibration Capture Node Destroyed");
}

void CameraCalibrationCapture::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 转换ROS图像消息为OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image_ = cv_ptr->image.clone();
        
        // 在图像上添加说明文字
        cv::Mat display_image = current_image_.clone();
        cv::putText(display_image, "Press SPACE to capture, ESC to exit", 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(display_image, "Images captured: " + std::to_string(image_count_), 
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        // 显示图像
        cv::imshow("Camera Calibration", display_image);
        
        // 处理按键
        int key = cv::waitKey(1) & 0xFF;
        if (key == 32) { // 空格键
            capture_image();
        } else if (key == 27) { // ESC键
            RCLCPP_INFO(this->get_logger(), "Exit requested by user");
            rclcpp::shutdown();
        }
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void CameraCalibrationCapture::capture_image() {
    if (current_image_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No image available to capture");
        return;
    }
    
    // 生成文件名
    std::stringstream filename;
    filename << save_directory_ << "/calibration_image_" 
            << std::setfill('0') << std::setw(4) << image_count_ << ".jpg";
    
    // 保存图像
    if (cv::imwrite(filename.str(), current_image_)) {
        image_count_++;
        RCLCPP_INFO(this->get_logger(), "Image saved: %s (Total: %d)", 
                   filename.str().c_str(), image_count_);
        
        // 显示保存成功的反馈
        cv::Mat feedback_image = current_image_.clone();
        cv::putText(feedback_image, "Image Saved!", 
                   cv::Point(current_image_.cols/2 - 100, current_image_.rows/2), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 3);
        cv::imshow("Camera Calibration", feedback_image);
        cv::waitKey(500); // 显示反馈0.5秒
        
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save image: %s", filename.str().c_str());
    }
}

} // namespace sensor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<sensor::CameraCalibrationCapture>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

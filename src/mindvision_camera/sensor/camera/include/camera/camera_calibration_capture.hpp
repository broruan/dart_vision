#ifndef CAMERA_CALIBRATION_CAPTURE_HPP_
#define CAMERA_CALIBRATION_CAPTURE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

namespace sensor {

class CameraCalibrationCapture : public rclcpp::Node {
public:
    explicit CameraCalibrationCapture(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CameraCalibrationCapture();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void capture_image();
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    cv::Mat current_image_;
    std::string save_directory_;
    int image_count_;
};

} // namespace sensor

#endif // CAMERA_CALIBRATION_CAPTURE_HPP_

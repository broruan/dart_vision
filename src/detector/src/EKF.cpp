#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "detector/EKF.hpp"


namespace EKF
{
    EKF_detector::EKF_detector(const std::string& node_name) : Node(node_name){
        // 订阅话题
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_pub",
            rclcpp::SensorDataQoS().keep_last(10), 
            std::bind(&EKF_detector::gainImg, this, std::placeholders::_1)
        );
    };
    EKF_detector::~EKF_detector() {
        cv::destroyAllWindows();
    };
    void EKF_detector::gainImg(const sensor_msgs::msg::Image::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "gainImg start!!!!!!!");

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image_ = cv_ptr->image.clone();
        // 用HSV提取颜色
        cv::Mat hsv, mask;
        cv::cvtColor(current_image_, hsv, cv::COLOR_BGR2HSV);

        // 红色 HSV 范围（根据实际灯光调整）
        cv::Scalar lower_red(160, 43, 46);
        cv::Scalar upper_red(180, 255, 255);
        cv::inRange(hsv, lower_red, upper_red, mask);
        cv::imshow("abstract", mask);
        cv::waitKey(1);
    };
} // namespace EKF

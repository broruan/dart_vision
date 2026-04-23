#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>

#include "detector/EKF.hpp"


namespace EKF
{
    EKF_detector::EKF_detector(const std::string& ndoe_name) : Node(ndoe_name){

    };
    void EKF_detector::gainImg(const sensor_msgs::msg::Image::SharedPtr msg){
        
    };
} // namespace EKF

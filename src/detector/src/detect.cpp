#include "detector/detect.hpp"

#include <vector>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std;

namespace detector{
    VideoDetectorNode::VideoDetectorNode(const string& node_name) : Node(node_name){
        RCLCPP_INFO(this->get_logger(),"start video_capture_nade");
        // 订阅图像话题
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_pub",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&VideoDetectorNode::CallBack, this, std::placeholders::_1));
        // 创建OpenCV窗口
        cv::namedWindow("Camera Calibration", cv::WINDOW_AUTOSIZE);
        }
        // 析构释放
        VideoDetectorNode::~VideoDetectorNode() {
            cv::destroyAllWindows();
            RCLCPP_INFO(this->get_logger(), "Video Capture Node Destroyed");
        }
    
    void VideoDetectorNode::CallBack(const sensor_msgs::msg::Image::SharedPtr msg){
        try {
            // 转换ROS图像消息为OpenCV Mat
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image.clone();
            
            // 在图像上添加说明文字
            cv::Mat display_image = current_image_.clone();
            cv::putText(display_image, "Press ESC to exit", 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(display_image, "Images captured", 
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
            // 显示图像
            cv::imshow("Camera Calibration", display_image);
        
            // 处理按键
            int key = cv::waitKey(1) & 0xFF;
            if (key == 27) { // ESC键
                RCLCPP_INFO(this->get_logger(), "Exit requested by user");
                rclcpp::shutdown();
            }
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    }
}

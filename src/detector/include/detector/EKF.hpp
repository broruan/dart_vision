#include <rclcpp/rclcpp.hpp>
#include <string>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>


namespace EKF {
    class EKF_detector : public rclcpp::Node {
        public:
            EKF_detector(const std::string& ndoe_name);
            ~EKF_detector();

        public:
            /**
             * @brief: 用于处理飞镖图像
             */
            void gainImg(const sensor_msgs::msg::Image::SharedPtr msg);
            cv::Mat current_image_ ;

        protected:
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    };

}
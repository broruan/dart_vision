#include <rclcpp/rclcpp.hpp>
#include <string>


namespace EKF {
    class EKF_detector : public Node {
        public:
            EKF_detector(const std::string& ndoe_name);
            ~EKF_detector();

        public:
            /**
             * @brief: 用于处理飞镖图像
             */
            void gainImg(const sensor_msgs::msg::Image::SharedPtr msg);

        protected:

    }

}
#ifndef DETECTOR_DETECT_HPP_
#define DETECTOR_DETECT_HPP_

#include <vector>
#include <string>
#include <memory>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <communicate_2025/msg/serial_info.hpp>
#include <detector/msg/deal_img.hpp>
#include <communicate_2025/msg/autoaim.hpp>
using namespace std;

namespace detector {
  // 初始化相机内参矩阵（50mm）
cv::Mat  cameraMatrix_2 = (cv::Mat_<double>(3, 3) << 
26957.642421, 0.000000, 1185.602897,
0.000000, 27505.347974, 763.496871,
0.000000, 0.000000, 1.000000);
// 26957.642421 0.000000 1185.602897
// 0.000000 27505.347974 763.496871
// 0.000000 0.000000 1.000000

cv::Mat distCoeffs_2 = (cv::Mat_<double>(1, 5) << 
0.516379, -72.850855, -0.028509, 0.008418, 0.000000);
// 0.516379 -72.850855 -0.028509 0.008418 0.000000

  // 初始化内参矩阵(8mm)
cv::Mat cameraMatrix_1 = (cv::Mat_<double>(3,3) << 
2217.955389, 0.000000, 649.805244,
0.000000, 2215.592248, 560.102413,
0.000000, 0.000000, 1.000000
);

//初始化畸变系数(8mm)
cv::Mat distCoeffs_1 = (cv::Mat_<double>(1,5) << 
    -0.092404, 1.743791, 0.009456, 0.008387, 0.000000);
// cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 
//   6351.668421, 0.000000, 615.282519,
//   0.00,        6339.565574, 586.540408,
//   0.00,        0.000,       1.0);

// cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 
//   -0.027849, 0.421400, 0.002825, -0.002343, 0.00);





/**
 * @brief 视频流检测节点
 *
 * 订阅图像话题、执行检测并发布检测结果或可视化图像
 */
class VideoDetectorNode : public rclcpp::Node {
  /**
   * @brief 定义配置文件变量
   */
  private:
    cv::Mat current_image_;
    double ch;
    int a_area;
    int a_area_2;
    float LIGHT_RADIUS;           //识别半径
    double mass;                  // 物体质量 (kg)
    double area;                  // 迎风面积 (m^2)
    double cd;                    // 风阻系数
    double rho;                   // 空气密度 (kg/m^3)
    int hsv_;                     // 是否用hsv
    bool debug_;                  // 调试模式
    double d_yaw;                 // yaw角偏差 8mm
    double d_yaw_2;		  // 50mm
    uint8_t distance = 30;             // 下位机距离消息
    uint8_t count;                // 第几发镖
    double f;
    double cx;
    double dist;
    double yaw_2;

 public:
  /**
   * @brief 构造函数
   *
   * @param node_name 节点名称
   * @param options 节点选项
   */
    explicit VideoDetectorNode(const string& node_name);
    ~VideoDetectorNode();
  /**
   * @brief 用于图像处理的成员变量
   */
    cv::Mat pre_img;


 protected:
  /**
   * @brief 用于处理速度的回调
   */
  void CallBack(const detector::msg::DealImg::SharedPtr msg);
  /**
   * @brief 图像处理函数
   */
  void dealImg(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief 接收下位机数据
   */
  void Serial_Recv(const communicate_2025::msg::Autoaim msg);


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<detector::msg::DealImg>::SharedPtr vel_sub_;
  rclcpp::Subscription<communicate_2025::msg::Autoaim>::SharedPtr serial_recv_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<detector::msg::DealImg>::SharedPtr result_pub_;
  rclcpp::Publisher<communicate_2025::msg::SerialInfo>::SharedPtr serial_pub_;
  // rclcpp::Subscription<communicate_2025::msg::SerialInfo>::SharedPtr serial_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
  int flag;
  // 缓存速度信息
  std::optional<double> cached_velocity_;
  std::optional<double> cached_s_;
};


}  // namespace detector

#endif  // DETECTOR_DETECT_HPP_

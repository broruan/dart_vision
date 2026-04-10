#ifndef DETECTOR_DETECT_HPP_
#define DETECTOR_DETECT_HPP_

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

namespace detector {
  // 初始化内参矩阵
cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 
    1776.435957, 0.000000, 608.647463,
    0.000000, 1771.549775, 503.873291,
    0.000000, 0.000000, 1.000000);

// 初始化畸变系数
cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 
    -0.014475125268044889, 0.10883076621917726, -0.0100935289275683, 0.01123908464042061, 0.0);

/**
 * @brief 检测结果结构体
 * 
 * 用于表示单个目标检测结果
 */
struct DetectionResult {
  /// 目标框左上角x坐标
  float x;
  /// 目标框左上角y坐标
  float y;
  /// 目标框宽度
  float width;
  /// 目标框高度
  float height;
  /// 目标yaw角
  float yaw;
  /// 目标pitch角
  float pitch;
  /// 置信度分数 (0.0 - 1.0)
  float confidence;
  /// 类别ID
  int class_id;
  /// 类别名称
  std::string class_name;

  DetectionResult() 
    : x(0), y(0), width(0), height(0), confidence(0.0), class_id(-1) {}

  DetectionResult(float x, float y, float w, float h, float conf, int cls_id)
    : x(x), y(y), width(w), height(h), confidence(conf), class_id(cls_id) {}
};
cv::Mat current_image_;

/**
 * @brief 检测器配置结构体
 * 
 * 用于配置检测器的参数
 */
struct DetectorConfig {
  /// 置信度阈值
  float confidence_threshold = 0.5f;
  /// NMS (Non-Maximum Suppression) IOU阈值
  float nms_threshold = 0.4f;
  /// 输入图像预期宽度
  int input_width = 416;
  /// 输入图像预期高度
  int input_height = 416;
  /// 是否进行类别过滤
  bool enable_class_filter = false;
  /// 要检测的类别ID列表
  std::vector<int> filter_class_ids;
};

/**
 * @brief 目标检测器基类
 * 
 * 提供基础的目标检测功能，支持多种深度学习框架
 */

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
    double ch;
    int a_area;
    float LIGHT_RADIUS;
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

  /**
   * @brief 初始化检测器
   *
   * @param model_path 模型文件路径
   * @param config_path 配置文件路径（YOLO等）
   * @param names_path 类别名称文件路径
   * @param config 检测器配置
   * @return 是否初始化成功
   */
  bool InitializeDetector(const std::string& model_path,
                         const std::string& config_path,
                         const std::string& names_path,
                         const DetectorConfig& config);

//   /**
//    * @brief 设置输入图像话题
//    */
//   void SetInputImageTopic(const std::string& topic);

//   /**
//    * @brief 设置输出图像话题
//    */
//   void SetOutputImageTopic(const std::string& topic);

//   /**
//    * @brief 设置检测结果文本发布话题
//    */
//   void SetDetectionTextTopic(const std::string& topic);

//   /**
//    * @brief 检查节点是否已准备好
//    */
//   bool IsReady() const;

 protected:
  /**
   * @brief 图像回调函数
   */
  void CallBack(const sensor_msgs::msg::Image::SharedPtr msg);
  /**
   * @brief 图像处理函数
   */
  void dealImg(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief 将 OpenCV 图像转换为 ROS2 图像消息,其实感觉不用
   */
  sensor_msgs::msg::Image::UniquePtr ConvertMatToImageMsg(const cv::Mat& image,
                                                        const std::string& encoding = "bgr8") const;

  /**
   * @brief 将检测结果绘制到图像上
   */
  cv::Mat DrawDetections(const cv::Mat& image,
                        const std::vector<DetectionResult>& detections) const;

  /**
   * @brief 将检测结果转换为文本字符串
   */
  std::string DetectionsToString(const std::vector<DetectionResult>& detections) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
};


}  // namespace detector

#endif  // DETECTOR_DETECT_HPP_

#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include "mindvision.hpp"
#include "inner_shot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "communicate_2025/msg/autoaim.hpp"
#include "detector/msg/deal_img.hpp"
#include <opencv2/videoio.hpp>
#include <mutex>
#include <shared_mutex>

namespace sensor {

struct ROI{
    int x = 0;
    int y = 0;
    int w = 720;
    int h = 720;
};

class CameraNode : public rclcpp::Node
{
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);
    void init_8mm();
    void init_50mm();

private:
    /**
     * @brief 发布图像
     */
    void LoopForPublish();

    /**
     * @brief 调整曝光
     */
    void TuneExposure();

    /**
     * @brief 通过trackbar设置曝光时间
     */
    void SetExposureTimeWithTrack(const double exposure_time_max);

    /**
     * @brief 保存曝光参数
     */
    // void CameraNode::ExposureCallback();

    /**
     * @brief 获取图像保存到 frame，从相机或者视频流
     */
    void GetImg();

    /**
     * @brief 开启内录节点
     */
    void InnerShot();

    /**
     * @brief 发布相机内参
     */

    void PublicCameraInfo();


    void timer();

    // 保存从摄像头获取的图像
    std::shared_ptr<cv::Mat> frame_;
    std::shared_ptr<cv::Mat> frame2_;  // 第二个相机帧缓存

    std::mutex frame_mutex_;  // 保护帧数据访问

    std::string enemy_color_flag_;
//TODO:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_for_radar_;

    // 是否外部输入视频流标志位
    bool videoflag;
    std::string video_path;
    std::shared_ptr<MindVision> mindvision_; // 8mm
    cv::VideoCapture capture;
    std::thread thread_for_publish_;    //获取图像的线程
    std::thread thread_for_inner_shot_; //获取图像的线程
    bool inner_shot_flag;
    int exposure_time;
    int gain;
    bool rosbag_flag;
    // double yaw;      // 接收yaw

    bool debug_exposure = false;
    int failed_count;
    int failed_count2;
    int flag;
    std::atomic<int> frame_count_{0};
    std::atomic<int> frame_count2_{0};

    // 第二个相机相关
    std::shared_ptr<MindVision> mindvision2_;  // 50mm

    std::thread thread_for_capture2_;  // 50mm相机读取线程
    std::thread thread_for_capture1_;  // 8mm相机线程
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::Subscription<communicate_2025::msg::Autoaim>::SharedPtr serial_sub_;

    void sub_callback(const communicate_2025::msg::Autoaim::SharedPtr msg);
    void callback_res(const detector::msg::DealImg::SharedPtr msg);
    std::atomic<uint8_t> count{0};    // 目前发射的飞镖数
    std::atomic<uint8_t> distance{30}; // 目标距离
    std::atomic<double> yaw{1000.0f};

    bool use_camera2 = false;  // 是否启用50mm相机

    // 订阅话题
    rclcpp::Subscription<detector::msg::DealImg>::SharedPtr sub_res_;
};

} // namespace sensor

#endif // CAMERA_NODE_HPP


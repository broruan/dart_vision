#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include "camera/mindvision.hpp"
#include "inner_shot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "communicate_2025/msg/autoaim.hpp"
#include <opencv2/videoio.hpp>
#include <rclcpp/subscription.hpp>

namespace sensor {

class CameraNode: public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);
    ~CameraNode() override;

private:
    /**
     * @brief 发布图像
     */
    void LoopForPublish();

    /*
     * @brief 自喵模式切换回调函数
     * @param 
     */
    void AutoAimSwitch(const communicate_2025::msg::Autoaim::SharedPtr auto_aim_mode_swicth_msg); 

    /**
     * @brief 获取图像保存到 frame，从相机或者视频流
     */
    void GetImg();

    /**
     * @brief 开启内录节点
     */
    void InnerShot();

    // 保存从摄像头获取的图像
    std::shared_ptr<cv::Mat> frame_;

    rclcpp::Subscription<communicate_2025::msg::Autoaim>::SharedPtr debug_auto_aim_mode_sub_;
    rclcpp::Subscription<communicate_2025::msg::Autoaim>::SharedPtr auto_aim_mode_sub_;
    std::string enemy_color_or_rune_flag;
    bool mode_; //决定图片往哪个topic发布 符或者装甲板 mode: rune true auto_aim false
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_for_rune_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_for_armor_;
    int rune_use_exposure_;

    // 是否外部输入视频流标志位
    bool videoflag;
    std::string video_path;
    std::shared_ptr<MindVision> mindvision_;
    cv::VideoCapture capture;
    std::thread thread_for_publish_;    // 获取图像的线程
    std::thread thread_for_inner_shot_; // 内录线程
    bool inner_shot_flag;

    int failed_count;

    std::string camera_config_path_;
    std::string rune_camera_config_path_;
    std::string sn_;
};

} // namespace sensor

#endif // CAMERA_NODE_HPP

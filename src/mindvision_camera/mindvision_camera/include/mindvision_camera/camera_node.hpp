#ifndef __CAMERA_NODE_HPP__
#define __CAMERA_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "mindvision_camera/mindvision_camera.hpp"
#include "mindvision_camera/thread_pool/thread_pool.hpp"

using std::string;

std::string mat_type2encoding(int mat_type);

namespace camera
{
// 采用双继承，是因为想让参数赋值更方便
class CameraNode : public rclcpp::Node, public Mindvision
{
public:
    CameraNode(const string & name, bool intra_process_comm = true);
    CameraNode(const rclcpp::NodeOptions& options);
    ~CameraNode();

private:
    /* 线程池 */
    //TODO: 我写的线程池之后应该是用不太上，但是测试玩玩。
    //TODO: ROS 有自己的多线程实现方式，也就是多线程执行器和回调函数组，之后应该还是用回这个。
    //std::shared_ptr<ThreadPool> thread_pool_;

    /* 主要功能：图像发布 相关 */
    std::map<std::string, cv::Mat> frame_;      //TIP: 该成员变量现在是为了给录制而定义
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pub_;
    void image_publish_(const std::string & camera_name);

    std::map<std::string, rclcpp::TimerBase::SharedPtr> cam_task_timer_;
    void cam_task_(const std::string & name);

        /* 视频录制 */
    std::map<std::string, cv::VideoWriter> video_recorder_;
    std::map<std::string, bool> video_record_need_flag_;
    std::string get_local_time();
    void video_record_(const std::string & camera_name);

    /* 参数相关 */
    // 工具函数
    std::vector<std::string> param_name_split_(const std::string & name);
    // 参数初始化
    void ParamInit_();
    void on_camera_parameter_init_(const std::string & name);
    std::set<std::string> camera_name_set;
    // 参数设置限制
    rcl_interfaces::msg::SetParametersResult ParamSetCallback_(std::vector<rclcpp::Parameter> parameters);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_set_callhand_;      // 参数要求设置的回调句柄
    std::map<std::string, double> trigger_fps, exposure_time;      // 用于存遍历完发现要重载的exposure.time和trigger.fps参数，因为想要每个相机独立赋参，所以用了关联容器
    std::map<std::string, bool> record_init_flag_;      // 录制使能参数是否有初始化
    // 参数与类交互事件
    bool on_parameter_event_(rcl_interfaces::msg::ParameterEvent::UniquePtr & event, rclcpp::Logger logger);
    void ParamEventCallback_(rcl_interfaces::msg::ParameterEvent::UniquePtr event);
    rclcpp::SyncParametersClient::SharedPtr param_event_client_;        // 参数有过设置之后事件的回调句柄
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;

    /* 参数测试用定时器 */
    rclcpp::TimerBase::SharedPtr timer_;
    void on_time();
};

}

#endif
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
        // 声明参数并设置默认值
        this->ch = this->declare_parameter("circularity_thresh",0.2);
        this->a_area = this->declare_parameter("aim_area",200);
        this->LIGHT_RADIUS = this->declare_parameter("LIGHT_RADIUS",0.03);
        // 订阅图像话题
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_pub",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&VideoDetectorNode::dealImg, this, std::placeholders::_1));
        // 创建OpenCV窗口
        cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
        }
        // 析构释放
        VideoDetectorNode::~VideoDetectorNode() {
            cv::destroyAllWindows();
            RCLCPP_INFO(this->get_logger(), "Video Capture Node Destroyed");
        }
    
    
    void VideoDetectorNode::dealImg(const sensor_msgs::msg::Image::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),"Img read successfully!Start to deal!");
    try{
        // 转换ROS图像消息为OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image_ = cv_ptr->image.clone();
         // ─────────────────────────────────────────
    // 1. 绿灯物理尺寸（已知）
    //    以圆心为原点，建立物体坐标系
    //    圆在 XY 平面内，Z=0
    // ─────────────────────────────────────────
    // 用圆上均匀分布的8个点作为3D特征点
    std::vector<cv::Point3f> object_points;
    int N = 8;
    for (int i = 0; i < N; i++) {
        double angle = 2.0 * CV_PI * i / N;
        object_points.emplace_back(
            this->LIGHT_RADIUS * std::cos(angle),
            this->LIGHT_RADIUS * std::sin(angle),
            0.0f
        );
    }

    // ─────────────────────────────────────────
    // 2. HSV 颜色空间过滤绿色
    // ─────────────────────────────────────────
    cv::Mat hsv, mask;
    cv::cvtColor(current_image_, hsv, cv::COLOR_BGR2HSV);

    // 绿色 HSV 范围（根据实际灯光调整）
    cv::Scalar lower_green(40, 50, 50);
    cv::Scalar upper_green(80, 255, 255);
    cv::inRange(hsv, lower_green, upper_green, mask);

    // 形态学处理：先开运算去噪，再闭运算补孔
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel, {-1,-1}, 2);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, {-1,-1}, 2);

    // ─────────────────────────────────────────
    // 3. 轮廓检测 + 圆形度筛选
    // ─────────────────────────────────────────
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::RotatedRect best_ellipse;
    bool found = false;

    for (auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < this->a_area) continue; // 面积过滤，去掉噪点

        double perimeter = cv::arcLength(contour, true);
        if (perimeter < 1e-5) continue;

        // 圆形度：4π·A / P?，越接近1越圆
        double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        if (circularity < this->ch) continue; // 阈值可调

        // 至少需要5个点才能拟合椭圆
        if (contour.size() < 5) continue;

        // 长短轴比例检查（透视下不会过于扁）
        cv::RotatedRect ellipse = cv::fitEllipse(contour);
        float ratio = std::min(ellipse.size.width, ellipse.size.height) /
                      std::max(ellipse.size.width, ellipse.size.height);
        if (ratio < 0.5f) continue;

        // 选面积最大的目标
        if (!found || area > cv::contourArea(contours[0])) {
            best_ellipse = ellipse;
            found = true;
        }
    }

    if (!found) {
        RCLCPP_WARN(this->get_logger(), "No green light detected.");
        return;
    }

    // ─────────────────────────────────────────
    // 4. 从拟合椭圆提取8个图像点（与3D点对应）
    // ─────────────────────────────────────────
    std::vector<cv::Point2f> image_points;
    float a = best_ellipse.size.width  / 2.0f; // 长半轴
    float b = best_ellipse.size.height / 2.0f; // 短半轴
    float angle_deg = best_ellipse.angle;
    cv::Point2f center = best_ellipse.center;

    for (int i = 0; i < N; i++) {
        double t = 2.0 * CV_PI * i / N;
        // 椭圆参数方程（含旋转角）
        double rad = angle_deg * CV_PI / 180.0;
        float px = center.x + a * std::cos(t) * std::cos(rad)
                             - b * std::sin(t) * std::sin(rad);
        float py = center.y + a * std::cos(t) * std::sin(rad)
                             + b * std::sin(t) * std::cos(rad);
        image_points.emplace_back(px, py);
    }

    // ─────────────────────────────────────────
    // 5. solvePnP 求解位姿
    // ─────────────────────────────────────────
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(
        object_points,
        image_points,
        cameraMatrix,
        distCoeffs,
        rvec, tvec,
        false,
        cv::SOLVEPNP_ITERATIVE
    );

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "solvePnP failed.");
        return;
    }

    // ─────────────────────────────────────────
    // 6. 计算 distance / yaw / pitch
    // ─────────────────────────────────────────
    // tvec 是相机坐标系下目标的平移向量 [tx, ty, tz]
    double tx = tvec.at<double>(0);
    double ty = tvec.at<double>(1);
    double tz = tvec.at<double>(2);

    // 欧氏距离
    double distance = std::sqrt(tx*tx + ty*ty + tz*tz);

    // Yaw：目标在相机水平方向的偏角（绕Y轴）
    double yaw   = std::atan2(tx, tz) * 180.0 / CV_PI;

    // Pitch：目标在相机竖直方向的偏角（绕X轴，注意Y轴朝下）
    double pitch = std::atan2(-ty, tz) * 180.0 / CV_PI;

    RCLCPP_INFO(this->get_logger(),
        "Distance: %.3f m | Yaw: %.2f deg | Pitch: %.2f deg",
        distance, yaw, pitch);

    // ─────────────────────────────────────────
    // 7. 可视化（调试用）
    // ─────────────────────────────────────────
    cv::ellipse(current_image_, best_ellipse, {0, 255, 0}, 2);
    cv::circle(current_image_, center, 4, {0, 0, 255}, -1);

    // 投影3D轴到图像平面，验证解算结果
    std::vector<cv::Point3f> axis_points = {
        {0,0,0}, {0.1f,0,0}, {0,0.1f,0}, {0,0,0.1f}
    };
    std::vector<cv::Point2f> projected;
    cv::projectPoints(axis_points, rvec, tvec,
                      cameraMatrix, distCoeffs, projected);
    cv::arrowedLine(current_image_, projected[0], projected[1], {0,0,255},   2); // X 红
    cv::arrowedLine(current_image_, projected[0], projected[2], {0,255,0},   2); // Y 绿
    cv::arrowedLine(current_image_, projected[0], projected[3], {255,0,0},   2); // Z 蓝

    std::string info = cv::format("D:%.2fm Y:%.1f P:%.1f", distance, yaw, pitch);
    cv::putText(current_image_, info, {0, 150},
                cv::FONT_HERSHEY_SIMPLEX, 3, {255,255,0}, 3);

    cv::imshow("Green Light Detection", current_image_);
    cv::waitKey(1);
}

    
        
catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    
    }


    // void VideoDetectorNode::CallBack(const sensor_msgs::msg::Image::SharedPtr msg){
    //     try {
    //         // 转换ROS图像消息为OpenCV Mat
    //         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //         current_image_ = cv_ptr->image.clone();
    //         // 此图像用于图像处理
    //         this->pre_img = current_image_.clone();
            
    //         // 在图像上添加说明文字
    //         cv::Mat display_image = current_image_.clone();
    //         cv::putText(display_image, "Press ESC to exit", 
    //                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    //         cv::putText(display_image, "Images captured", 
    //                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
    //         // 显示图像
    //         cv::imshow("Camera Calibration", display_image);
        
    //         // 处理按键
    //         int key = cv::waitKey(1) & 0xFF;
    //         if (key == 27) { // ESC键
    //             RCLCPP_INFO(this->get_logger(), "Exit requested by user");
    //             rclcpp::shutdown();
    //         }
}

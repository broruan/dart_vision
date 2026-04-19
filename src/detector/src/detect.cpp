#include "detector/detect.hpp"
#include "detector/orbit_calculate.hpp"

#include <vector>
#include <string>
#include <memory>
#include <optional>
#include <cmath>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <communicate_2025/msg/serial_info.hpp>

// using namespace std;
// 需要写进yaml中的参数：HSV颜色范围、可选：开和闭运算的kernel、拟合圆半径误差
namespace detector{
    VideoDetectorNode::VideoDetectorNode(const string& node_name) : Node(node_name){



        RCLCPP_INFO(this->get_logger(),"start video_capture_nade");
        // 声明参数并设置默认值
        this->ch = this->declare_parameter("circularity_thresh",0.7);
        this->a_area = this->declare_parameter("aim_area",200);
        this->LIGHT_RADIUS = this->declare_parameter("LIGHT_RADIUS",0.03);
        this->mass = this->declare_parameter("mass",0.20);
        this->area = this->declare_parameter("area",0.001);
        this->cd = this->declare_parameter("cd",0.3);
        this->rho = this->declare_parameter("rho",1.255);



        // 订阅图像话题
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_pub",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&VideoDetectorNode::dealImg, this, std::placeholders::_1));

        vel_sub_ = this->create_subscription<detector::msg::DealImg>(
            "/vel_pub",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&VideoDetectorNode::CallBack, this, std::placeholders::_1));


        // serial_sub_ = this->create_subscription<communicate_2025::msg::SerialInfo>("/detect_info", rclcpp::SystemDefaultsQoS(),
        //     std::bind(&VideoDetectorNode::Callback, this, std::placeholders::_1))

        } 
        // 析构释放
        VideoDetectorNode::~VideoDetectorNode() {
            cv::destroyAllWindows();
            RCLCPP_INFO(this->get_logger(), "Video Capture Node Destroyed");
        }
    
    void VideoDetectorNode::dealImg(const sensor_msgs::msg::Image::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),"Img read successfully!Start to deal!");
    try{
        detector::msg::DealImg result;
        communicate_2025::msg::SerialInfo msg_to_serial;
        // 转换ROS图像消息为OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image_ = cv_ptr->image.clone();

         // ─────────────────────────────────────────
    // 1. 绿灯物理尺寸（已知）
    //    以圆心为原点，建立物体坐标系
    //    圆在 XY 平面内，Z=0
    // ─────────────────────────────────────────
    // 用圆上均匀分布的8个点作为3D特征点
    // std::vector<cv::Point3f> object_points;
    // int N = 8;
    // for (int i = 0; i < N; i++) {
    //     double angle = 2.0 * CV_PI * i / N;
    //     object_points.emplace_back(
    //         (this->LIGHT_RADIUS) * std::cos(angle),
    //         (this->LIGHT_RADIUS) * std::sin(angle),
    //         0.0f
    //     );
    // }

    // ─────────────────────────────────────────
    // 2. HSV 颜色空间过滤绿色
    // ─────────────────────────────────────────
    cv::Mat hsv, mask;
    cv::cvtColor(current_image_, hsv, cv::COLOR_BGR2HSV);

    // 绿色 HSV 范围（根据实际灯光调整）
    cv::Scalar lower_green(40, 50, 35);
    cv::Scalar upper_green(80, 255, 255);
    cv::inRange(hsv, lower_green, upper_green, mask);

    // 形态学处理：先开运算去噪，再闭运算补孔
    cv::Mat kernel_1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size2f{1, 1});// 开运算
    cv::Mat kernel_2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size2f{5, 5});// 闭运算
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel_1, {-1,-1}, 2);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_2, {-1,-1}, 2);

    // ─────────────────────────────────────────
    // 3. 轮廓检测 + 圆形度筛选
    // ─────────────────────────────────────────
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 调试用，画出轮廓
    cv::Mat temp = cv::Mat::zeros(mask.size(),CV_8UC3);
    cv::drawContours(temp, contours, -1, cv::Scalar(255, 255, 255), 1);
    cv::imshow("contours", temp);


    cv::RotatedRect best_ellipse;
    cv::RotatedRect ellipses;
    std::vector<cv::Point> true_contour;

    bool found = false;
    double min_area = 100000;

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

        // 得到筛选后的轮廓
        true_contour = contour;

        // 长短轴比例检查（透视下不会过于扁）
        ellipses = cv::fitEllipse(contour);
        float ratio = std::min(ellipses.size.width, ellipses.size.height) /
                      std::max(ellipses.size.width, ellipses.size.height);
        if (ratio < 0.5f) continue;

        // 选面积最小的目标
        if (!found || (area < min_area)) {
            min_area = area;
            best_ellipse = ellipses;
            found = true;
        }
    }
    // std::cout << "Image size: " << current_image_.size() << std::endl;

    // if (!found) {
    //     RCLCPP_WARN(this->get_logger(), "No green light detected.");
    //     return;
    // }
    if(best_ellipse.size.width != 0){

    // ─────────────────────────────────────────
    // 4. 从拟合椭圆提取8个图像点（与3D点对应）
    // ─────────────────────────────────────────
    // std::vector<cv::Point2f> image_points;
    // float a = ellipses.size.width  / 2.0f; // 长半轴,原本用best_ellipse，现在用ellipse
    // float b = ellipses.size.height / 2.0f; // 短半轴
    // float angle_deg = ellipses.angle;
    // cv::Point2f center = ellipses.center;

    // for (int i = 0; i < N; i++) {
    //     double t = 2.0 * CV_PI * i / N;
    //     // 椭圆参数方程（含旋转角）
    //     double rad = angle_deg * CV_PI / 180.0;
    //     float px = center.x + a * std::cos(t) * std::cos(rad)
    //                          - b * std::sin(t) * std::sin(rad);
    //     float py = center.y + a * std::cos(t) * std::sin(rad)
    //                          + b * std::sin(t) * std::cos(rad);
    //     image_points.emplace_back(px, py);
    // }
    
    /*
    重写solvepnp中2D,3D对应点的代码
    直接用轮廓上的点
    不拟合椭圆  
    */
        const int N = 8;
        std::vector<cv::Point2f> image_points;
        std::vector<cv::Point3f> object_points;
        // 用椭圆中心估计圆心的位置
        cv::Point2f center_2d = best_ellipse.center;


        // 取8个点
        for(int i=0; i<N; ++i){
            double theta = 2.0 * CV_PI * i / N;  // 均匀取 N 个角度                                                                                                                                                
            float a = best_ellipse.size.width  / 2.0f;                                                                                                                                                             
            float b = best_ellipse.size.height / 2.0f;
            float angle_rad = best_ellipse.angle * CV_PI / 180.0f;// 转成弧度                                                                                                                                                 
                                                                                                                                                                                                             
            // 椭圆参数方程，计算椭圆边界上的点（已旋转）                                                                                                                                                          
            float px = best_ellipse.center.x + a * std::cos(theta) * std::cos(angle_rad)
                                      - b * std::sin(theta) * std::sin(angle_rad);                                                                                                                           
            float py = best_ellipse.center.y + a * std::cos(theta) * std::sin(angle_rad)
                                      + b * std::sin(theta) * std::cos(angle_rad);                                                                                                                           
   
            image_points.push_back(cv::Point2f(px, py)); 
            // 对应的3D点：从圆心方向推断该点在圆上的角度
            // 用该点相对椭圆中心的角度，映射回3D圆的角度
            float dx = px - center_2d.x;
            float dy = py - center_2d.y;

            // 去掉椭圆旋转角的影响，还原到对齐坐标系
            float dx_local =  dx * std::cos(angle_rad) + dy * std::sin(angle_rad);
            float dy_local = -dx * std::sin(angle_rad) + dy * std::cos(angle_rad);

            // 归一化后得到3D圆上的角度
            float theta_3D = std::atan2(dy_local / b, dx_local / a);

            object_points.emplace_back(
                this->LIGHT_RADIUS * std::cos(theta_3D),
                this->LIGHT_RADIUS * std::sin(theta_3D),
                0.0f
            );
        }

        float radius_img = (std::min(best_ellipse.size.height, best_ellipse.size.width)) / 2.0f;
        double f = cameraMatrix.at<double>(0,0); // fx
        double dist = (f * (this->LIGHT_RADIUS) / radius_img) * 2; 


    // ─────────────────────────────────────────
    // 5. solvePnP 求解位姿
    // ─────────────────────────────────────────
    cv::Mat rvec, tvec;
    cv::solvePnP(
        object_points,
        image_points,
        cameraMatrix,
        distCoeffs,
        rvec, tvec,
        false,
        cv::SOLVEPNP_ITERATIVE
    );

    // if (!success) {
    //     RCLCPP_WARN(this->get_logger(), "solvePnP failed.");
    //     return;
    // }

    // ─────────────────────────────────────────
    // 6. 计算 distance / yaw / pitch
    // ─────────────────────────────────────────
    // tvec 是相机坐标系下目标的平移向量 [tx, ty, tz]
    double tx = tvec.at<double>(0);
    double ty = tvec.at<double>(1);
    double tz = tvec.at<double>(2);

    // 欧氏距离
    double distance = std::sqrt(tx*tx + ty*ty + tz*tz);

    // Yaw：目标在相机水平方向的偏角（绕Y轴），右正左负
    double yaw   = std::atan2(tx, tz) * 180.0 / CV_PI;

    // Pitch：目标在相机竖直方向的偏角（绕X轴，注意Y轴朝下），上正下负
    double pitch = std::atan2(-ty, tz) * 180.0 / CV_PI;

    
    // ─────────────────────────────────────────
    // 7. 可视化（调试用）
    // ─────────────────────────────────────────

    cv::ellipse(current_image_, best_ellipse, {0, 255, 0}, 2);
    cv::circle(current_image_, center_2d, 4, {0, 0, 255}, -1);

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
    result.pitch = pitch;
    result.distance = distance;
    result.yaw = yaw;
    result.found = 1;
    msg_to_serial.yaw = yaw;                                                                                                                                                                                           
    msg_to_serial.pitch = pitch;                                                                                                                                                                                         
    msg_to_serial.is_find.data = 1; // 注意！！！！！！！！！！！！！！
    }
    cv::imshow("Green Light Detection", current_image_);
    cv::imshow("color split", mask);
    cv::waitKey(1);
    result_pub_ = this->create_publisher<detector::msg::DealImg>("/deal_img",10);
    serial_pub_ = this->create_publisher<communicate_2025::msg::SerialInfo>("/detect_info", 10);
    result_pub_->publish(result);
    serial_pub_->publish(msg_to_serial);
}

    
        
catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    }

void VideoDetectorNode::CallBack(const detector::msg::DealImg::SharedPtr msg){
    // 发布话题
    serial_pub_ = this->create_publisher<communicate_2025::msg::SerialInfo>("/detect_info", 10);
    RCLCPP_INFO(this->get_logger(), "publish topic successfully!!!");

    communicate_2025::msg::SerialInfo msg_to_serial; 

    RCLCPP_INFO(this->get_logger(), "calculate successfully!!!");
    msg_to_serial.velocity = msg->velocity;    // 新增
    
    
    /*
        发布数据到下位机
    */                                                                                                                                                                                                                                                                                          
    serial_pub_->publish(msg_to_serial);
    RCLCPP_INFO(this->get_logger(), "serial publish complete!!!");

}

}// namespace detector

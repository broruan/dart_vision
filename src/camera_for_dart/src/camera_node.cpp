#include "camera/camera_node.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"
#include "communicate_2025/msg/autoaim.hpp"
#include <mutex>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unistd.h>

namespace sensor {

CameraNode::CameraNode(const rclcpp::NodeOptions& options):
    Node("camera_node", options),
    frame_(std::make_shared<cv::Mat>()),
    frame2_(std::make_shared<cv::Mat>()),
    failed_count(0),
    failed_count2(0) {
    RCLCPP_INFO(this->get_logger(), "camera_node start");

    // 订阅下位机消息
    serial_sub_ = this->create_subscription<communicate_2025::msg::Autoaim>(
        "/communicate/autoaim",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&CameraNode::sub_callback, this, std::placeholders::_1)
    ); 


    // 是否使用视频流标志位
    videoflag  = this->declare_parameter("videoflag", false);
    video_path = this->declare_parameter(
        "video_path",
        "/home/phoenix/zk/save_stuff/20.mp4"
    ); //默认路径
    rosbag_flag     = this->declare_parameter("rosbag_flag", false);
    inner_shot_flag = this->declare_parameter("inner_shot_flag", false);
    exposure_time   = this->declare_parameter("exposure_time", 5000);
    gain            = this->declare_parameter("gain", 64);
    rosbag_flag     = this->get_parameter("rosbag_flag").as_bool();

    RCLCPP_INFO(this->get_logger(), "inner_shot flag %d", inner_shot_flag);

    // 分别初始化8mm和50mm相机
   // CameraNode::init_8mm();
    CameraNode::init_50mm();


    if (rosbag_flag) {
        RCLCPP_INFO(this->get_logger(), "rosbag");
        return;
    }
    else if (videoflag) {
        capture.open(video_path);
        if (!capture.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "video open failed");
            exit(-1);
        }
        RCLCPP_INFO(this->get_logger(), "use video");
    }



    if (inner_shot_flag) {
        thread_for_inner_shot_ = std::thread(std::bind(&CameraNode::InnerShot, this));
    }

    img_pub_for_radar_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/raw_image",
        rclcpp::SensorDataQoS().keep_last(10)
    );

    // if (!videoflag) {
    //     CameraNode::SetExposureTimeWithTrack(exposure_time_max);
    // }
    // if (!videoflag) {
    //     CameraNode::TuneExposure();
    // }

    double exposure_time_value;
   // mindvision_->GetExposureTime(exposure_time_value);
    RCLCPP_DEBUG(this->get_logger(), "exposure_time: %f", exposure_time_value);
    // mindvision_->SetOnceWB();

    // thread_for_capture1_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));
    thread_for_capture1_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));
    thread_for_capture2_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));

    std::thread timerTread(std::bind(&CameraNode::timer, this));
    timerTread.detach();
}

void CameraNode::init_8mm() {
    mindvision_ = std::make_shared<MindVision>(
        ament_index_cpp::get_package_share_directory("camera_for_dart") + "/config/mindvision_8.config",
        this->declare_parameter("sn", "").c_str()
    );
    double exposure_time_max;
    double exposure_time_min;
    double exposure_time_step;
    mindvision_->GetExposureTimeRange(exposure_time_min, exposure_time_max, exposure_time_step);
    RCLCPP_INFO(
        this->get_logger(),
        "exposure_time_min: %f, exposure_time_max: %f, exposure_time_step: %f",
        exposure_time_min,
        exposure_time_max,
        exposure_time_step
    );
    if (!mindvision_->GetCameraStatus()) {
        RCLCPP_ERROR(this->get_logger(), "mindvision failed");
        exit(-1);
    }
    int wb_mode;
    mindvision_->GetWbMode(wb_mode);
    RCLCPP_INFO(this->get_logger(), "wb_mode(1为自动白平衡，0为手动): %d", wb_mode);
    if (wb_mode == 0) {
        int clr_temp_mode;
        mindvision_->GetClrTempMode(clr_temp_mode);
        RCLCPP_INFO(
            this->get_logger(),
            "ClrTmpMode(0为自动识别色温，1为指定预设，2为自定义): %d",
            clr_temp_mode
        );
        if (mindvision_->SetClrTempMode(1) != 0) {
            RCLCPP_ERROR(this->get_logger(), "SetClrTempMode failed");
        } else {
            mindvision_->GetClrTempMode(clr_temp_mode);
            RCLCPP_INFO(
                this->get_logger(),
                "ClrTmpMode(0为自动识别色温，1为指定预设，2为自定义): %d",
                clr_temp_mode
            );
        }
        int is_setWB = mindvision_->SetOnceWB();
        if (is_setWB == 0) {
            RCLCPP_INFO(this->get_logger(), "SetOnceWB success");
        } else {
            RCLCPP_ERROR(this->get_logger(), "SetOnceWB failed,failed code: %d", is_setWB);
        }
    }

    mindvision_->Overlay();

    mindvision_->SetExposureTime(exposure_time);
}


void CameraNode::init_50mm() {
    mindvision2_ = std::make_shared<MindVision>(
        ament_index_cpp::get_package_share_directory("camera_for_dart") + "/config/mindvision_50.config",
        this->declare_parameter("sn2", "041071320344").c_str()
    );
    double exposure_time_max;
    double exposure_time_min;
    double exposure_time_step;
    mindvision2_->GetExposureTimeRange(exposure_time_min, exposure_time_max, exposure_time_step);
    RCLCPP_INFO(
        this->get_logger(),
        "exposure_time_min: %f, exposure_time_max: %f, exposure_time_step: %f",
        exposure_time_min,
        exposure_time_max,
        exposure_time_step
    );
    if (!mindvision2_->GetCameraStatus()) {
        RCLCPP_ERROR(this->get_logger(), "mindvision2 failed");
        exit(-1);
    }
    int wb_mode;
    mindvision2_->GetWbMode(wb_mode);
    RCLCPP_INFO(this->get_logger(), "wb_mode(1为自动白平衡，0为手动): %d", wb_mode);
    if (wb_mode == 0) {
        int clr_temp_mode;
        mindvision2_->GetClrTempMode(clr_temp_mode);
        RCLCPP_INFO(
            this->get_logger(),
            "ClrTmpMode(0为自动识别色温，1为指定预设，2为自定义): %d",
            clr_temp_mode
        );
        if (mindvision2_->SetClrTempMode(1) != 0) {
            RCLCPP_ERROR(this->get_logger(), "SetClrTempMode failed");
        } else {
            mindvision2_->GetClrTempMode(clr_temp_mode);
            RCLCPP_INFO(
                this->get_logger(),
                "ClrTmpMode(0为自动识别色温，1为指定预设，2为自定义): %d",
                clr_temp_mode
            );
        }
        int is_setWB = mindvision2_->SetOnceWB();
        if (is_setWB == 0) {
            RCLCPP_INFO(this->get_logger(), "SetOnceWB success");
        } else {
            RCLCPP_ERROR(this->get_logger(), "SetOnceWB failed,failed code: %d", is_setWB);
        }
    }

    mindvision2_->Overlay();

    mindvision2_->SetExposureTime(exposure_time);
}

void CameraNode::InnerShot() {
    auto inner_shot = std::make_shared<InnerShotNode>();
    RCLCPP_INFO(this->get_logger(), "inner_shot start !.............. ");
    rclcpp::spin(inner_shot);
}

void CameraNode::GetImg() {
    std::unique_lock<std::mutex> lock(frame_mutex_);

    if (videoflag) {
        capture >> *frame_;
        // usleep(100000);

        // 循环播放
        if ((*frame_).empty()) {
            RCLCPP_INFO(this->get_logger(), "video end");
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            capture >> *frame_;
        }
    } else {
        std::cout << distance.load() << std::endl;
        if(this->distance.load() < 20) {
            if (!mindvision_->GetFrame(frame_)) {
                failed_count++;
                RCLCPP_ERROR(this->get_logger(), "mindvision get image failed");
            } else {
                frame_count_++;
                // RCLCPP_DEBUG(this->get_logger(), "mindvision get image
                // success. Size: %d x %d", frame_->cols, frame_->rows);
                failed_count = 0;    

            }
        } else {

            // 第二个相机同时读取

            if (!mindvision2_->GetFrame(frame_)) {
                failed_count2++;
                RCLCPP_ERROR(this->get_logger(), "mindvision2 get image failed");
            } else {
                frame_count2_++;
                failed_count2 = 0;
            }
        }
        
    }
    

    if (failed_count > 10) {
        exit(-1);
    }
    lock.unlock();
}

// void CameraNode::ExposureCallback(){
//     // if papam changed, save to file
//     if (this->has_parameter("exposure_time")) {
//         int exposure_time =
//         this->get_parameter("exposure_time").as_int();
//         mindvision_->SetExposureTime(exposure_time);
//     }
//     if (this->has_parameter("gain")) {
//         int gain = this->get_parameter("gain").as_int();
//         mindvision_->SetGain(gain);
//     }
// }

void CameraNode::TuneExposure() {
    CameraNode::GetImg();
    exposure_time = this->get_parameter("exposure_time").as_int();
    gain          = this->get_parameter("gain").as_int();

    // Setup the tune window and trackbars
    cv::namedWindow("tune", cv::WINDOW_NORMAL);

    cv::createTrackbar(
        "gain",
        "tune",
        NULL,
        128,
        [](int value, void* ptr) {
            auto mindvision = static_cast<MindVision*>(ptr);
            mindvision->SetGain(value);
        },
        mindvision_.get()
    );

    while (cv::waitKey(1) != 'q') {
        CameraNode::GetImg();
        cv::imshow("tune", *frame_);
    }
    cv::destroyAllWindows();
}

void CameraNode::SetExposureTimeWithTrack(const double exposure_time_max) {
    // Setup the tune window and trackbars
    cv::namedWindow("exposure_tune", cv::WINDOW_NORMAL);

    double exposure_time_value;
    mindvision_->GetExposureTime(exposure_time_value);

    int exposure_time_int = static_cast<int>(exposure_time_value);

    // 设备允许的曝光时间在0.008~10284.8ms之间，代码中单位为us
    cv::createTrackbar(
        "exposure_time",
        "exposure_tune",
        &exposure_time_int,
        static_cast<int>(exposure_time_max),
        [](int value, void* ptr) {
            auto mindvision = static_cast<MindVision*>(ptr);
            mindvision->SetExposureTime(value);

            double true_time;
            mindvision->GetExposureTime(true_time);
            std::cout << "Set true exposure_time: " << true_time << std::endl;
            // mindvision->SetOnceWB();
            std::cout << "reset white balance" << std::endl;
        },
        mindvision_.get()
    );

    while (cv::waitKey(1) != 'q') {
        CameraNode::GetImg();
        cv::imshow("exposure_tune", *frame_);
    }

    cv::destroyAllWindows();

    RCLCPP_INFO(this->get_logger(), "Set initial exposure_time: %d", exposure_time_int);
}

void CameraNode::LoopForPublish() {
    while (rclcpp::ok()) {
        this->GetImg();


        if (frame_->empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame, skip publishing.");
            continue;
        }

        // 发布全图
        {
            if (debug_exposure) {
                double exposure_time_value;
                mindvision_->GetExposureTime(exposure_time_value);
                RCLCPP_DEBUG(this->get_logger(), "exposure_time: %f", exposure_time_value);
            }
            auto full_msg =
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", *frame_).toImageMsg();
            full_msg->header.stamp    = this->now();
            full_msg->header.frame_id = this->enemy_color_flag_;
            img_pub_for_radar_->publish(*full_msg);
        }
    }
}

void CameraNode::timer() {
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        int count = frame_count_.exchange(0);
        // std::cout << "Camera FPS: " << count << std::endl;
    }
}

void CameraNode::sub_callback(const communicate_2025::msg::Autoaim::SharedPtr msg) {
    this->count.store(msg->count);
    this->distance.store(msg->distance); // 保证线程安全
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)

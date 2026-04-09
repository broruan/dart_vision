#include "mindvision_camera/camera_node.hpp"

std::string mat_type2encoding(int mat_type)
{
    switch(mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("是OpenCV不支持的编码格式呢");
    }
}

namespace camera
{

CameraNode::CameraNode(const string & name, bool intra_process_comm)
 : Node(name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comm))
{
    // 注册参数限制
    param_set_callhand_ = this->add_on_set_parameters_callback(std::bind(&CameraNode::ParamSetCallback_, this, std::placeholders::_1));
    // 注册参数赋值
    param_event_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
    param_event_sub_ = param_event_client_->on_parameter_event(std::bind(&CameraNode::ParamEventCallback_, this, std::placeholders::_1));
    // 参数初始化
    ParamInit_();

    // 注册图像输出发布
    for (const auto & name : camera_name_set) {
        image_pub_[name] = this->create_publisher<sensor_msgs::msg::Image>(name+"_image", 10);
        video_recorder_[name] = cv::VideoWriter();
        //TODO: 这个时间类型和硬件上的配合好像微妙的有些问题（可能）
        //TODO: 既然后面会要重开，那这里是否不用配置timer，毕竟这个timer马上就被取消掉了
        //TIP: 想要修改这里的回调lamda函数里的内容时，要注意这个timer可能会在ParamEvent里重开，要改改三遍，所以还是直接修改cam_task_()比较好
        cam_task_timer_[name] = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / 100.)), 
            std::function(
                [this, name] {
                    cam_task_(name);
                }
            )
        );
        cam_task_timer_[name]->cancel();
    }

    //TIP: 之前测试的遗留，现在他的回调函数已经被条件编译了，但这个有总全局意味的定时器之后应该还有其他用处
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(500ms, std::bind(&CameraNode::on_time, this));
}

CameraNode::CameraNode(const rclcpp::NodeOptions& options)
: Node("camera", options)
{
    // 注册参数限制
    param_set_callhand_ = this->add_on_set_parameters_callback(std::bind(&CameraNode::ParamSetCallback_, this, std::placeholders::_1));
    // 注册参数赋值
    param_event_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
    param_event_sub_ = param_event_client_->on_parameter_event(std::bind(&CameraNode::ParamEventCallback_, this, std::placeholders::_1));
    // 参数初始化
    ParamInit_();

    // 注册图像输出发布
    for (const auto & name : camera_name_set) {
        image_pub_[name] = this->create_publisher<sensor_msgs::msg::Image>(name+"_image", 10);
        video_recorder_[name] = cv::VideoWriter();
        //TODO: 这个时间类型和硬件上的配合好像微妙的有些问题（可能）
        //TODO: 既然后面会要重开，那这里是否不用配置timer，毕竟这个timer马上就被取消掉了
        //TIP: 想要修改这里的回调lamda函数里的内容时，要注意这个timer可能会在ParamEvent里重开，要改改三遍，所以还是直接修改cam_task_()比较好
        cam_task_timer_[name] = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / 100.)), 
            std::function(
                [this, name] {
                    cam_task_(name);
                }
            )
        );
        cam_task_timer_[name]->cancel();
    }
}

CameraNode::~CameraNode()
{
    for (auto & re : video_recorder_) {
        re.second.release();
    }
    cv::destroyAllWindows();
}

void CameraNode::cam_task_(const std::string & name)
{
    image_publish_(name);
    if (video_record_need_flag_[name]) {
        video_record_(name);
    }
}

void CameraNode::on_time()
{
    #if 0
        for (const auto & name : camera_name_set)
        RCLCPP_INFO_STREAM(
            this->get_logger(), 
            "相机名： " << std::setw(10) << name
            << "    exposure.time= " << std::setw(8) << cam_[name].exposure.time 
            << "    trigger.fps= " << std::setw(8) << cam_[name].trigger.fps;
        );
    #endif
}

void CameraNode::on_camera_parameter_init_(const std::string & name)
{
    // 录制使能
    this->declare_parameter(name+".record", false);
    // 图像尺寸
    this->declare_parameters<int>(name+".size", {{"width", 1280}, {"height", 1024}, {"Woffset", 0}, {"Hoffset", 0}});
    // 触发（要在曝光设置之前，因为高帧率需要低曝光）
    this->declare_parameter(name+".trigger.soft_enable", true);
    this->declare_parameter(name+".trigger.fps", 100.);
    // 曝光
    this->declare_parameter(name+".exposure.time", 1000.);
    this->declare_parameter(name+".exposure.auto_enable", false);
    this->declare_parameter(name+".exposure.auto_target", 60);
}

/**
 * @brief 负责节点的各种参数初始化，首先是和相机设备相关的参数
 */
void CameraNode::ParamInit_()
{
    /* 相机设备参数初始化 */
    // 打印连接的相机昵称
    for (const auto & c : cam_) {
        RCLCPP_INFO_STREAM(get_logger(), "Connected camera nickname: " << c.first);
    }
    // 先确定相机昵称列表
    auto camera_name_list = this->declare_parameter("camera_name_list", std::vector<string>{"yuki"});// "default","test", "example", 
    RCLCPP_INFO_STREAM(get_logger(), "Camera name list from config: ");
    for (const auto & name : camera_name_list) {
        RCLCPP_INFO_STREAM(get_logger(), "  " << name);
    }
    camera_name_set = std::set<std::string>(camera_name_list.begin(), camera_name_list.end());
    if(static_cast<std::size_t>(camera_amount) != camera_name_set.size()) {
        RCLCPP_INFO_STREAM(
            get_logger(), 
            "想要配置的相机数量是：" << camera_name_set.size() << "    "
            "实际连接找到的相机数量是：" << camera_amount
        );
    }
    std::vector<std::string> name_need_erase = {};  // 临时变量
    for (const auto & name : camera_name_set)
    {
        if(cam_.find(name) != cam_.end()) {
            RCLCPP_INFO_STREAM(get_logger(), "找到了需要配置的相机：" << name);
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), "没有找到需要配置的相机：" << name);
            name_need_erase.push_back(name);
        }
    }
    for (const auto & name : name_need_erase) {
        RCLCPP_WARN_STREAM(get_logger(), "因相机 " << name << " 没有被程序找到，将不会被配置");
        camera_name_set.erase(name);
    }
    for (const auto & cam : cam_) {
        if (camera_name_set.find(cam.first) == camera_name_set.end()) { 
            RCLCPP_ERROR_STREAM(get_logger(), "参数配置文件里没有相机 " << cam.first << " 的初始化配置参数");
            RCLCPP_WARN_STREAM(get_logger(), "将会用默认的参数为相机 " << cam.first << " 初始化");
            //TODO: 添加默认参数初始化
            camera_name_set.emplace(cam.first);
        }
    }

    // 确定参数文件所需的
    for(const auto & camera_name : camera_name_set) {
        RCLCPP_INFO_STREAM(get_logger(), "相机 " << camera_name << " 的配置参数将被初始化");
        on_camera_parameter_init_(camera_name);
    }
}

std::vector<std::string> CameraNode::param_name_split_(const std::string & name)
{
    std::vector<std::string> result;
    std::stringstream ss(name);
    std::string word;
    while (std::getline(ss, word, '.')) {
        result.push_back(word);
    }
    return result;
}

/**
 * @brief 负责限制容许的参数性质，阻止不合适的参数赋值
 */
rcl_interfaces::msg::SetParametersResult CameraNode::ParamSetCallback_(std::vector<rclcpp::Parameter> parameters)
{
    /**
     * 相机的参数：
     * 长（及其位移）限制 1280 ，宽（及其位移）限制 1024 。
     * 曝光时间不能超过要求的频率
     * 增益值要看SDK要求的范围
     */
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (auto parameter : parameters) {
        rclcpp::ParameterType parameter_type = parameter.get_type();
        std::string parameter_name = parameter.get_name();
        std::vector<std::string> param_name_vec = param_name_split_(parameter_name);
        if (rclcpp::ParameterType::PARAMETER_NOT_SET == parameter_type) {
            RCLCPP_INFO(
                this->get_logger(), "parameter '%s' deleted successfully", 
                parameter.get_name().c_str()
            );
            result.successful &= true;
        }

        // TODO: 检查是否有对应名称的相机硬件
        // 根据每个参数字段判断是哪个参数
        // 搞这么麻烦起因是为了给exposure.time和trigger.fps做限制
        // 相机节点只能相机参数有三字段
        if (3 ==  param_name_vec.size())
        {
            // 如果是曝光时间的设置，曝光时间不能超过帧率的限制
            // 同时提升帧率的话，也可能会需要曝光时间设置
            std::string camera_name =  param_name_vec[0];
            if (cam_.find(camera_name) == cam_.end()) {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "昵称名为 " << camera_name << 
                    " 的相机没有连接在设备上，因而无法设置名为 parameter_name 的参数"
                );
                result.reason = "can't find a camera on the computer is this name";
                result.successful = false;
                continue;
            }
            bool fps_or_time_flag = 0;  // 如果是曝光时间和帧率的量，后面要做处理
            if ("exposure" == param_name_vec[1]) {
                if ("time" == param_name_vec[2]) {
                    fps_or_time_flag = 1;
                    exposure_time[camera_name] = parameter.as_double();
                    #if 1
                        RCLCPP_INFO_STREAM(
                            get_logger(),
                            "得到了为相机 " << camera_name << " 准备的 exposure.time 参数" 
                        );
                    #endif
                }
            } else if ("trigger" == param_name_vec[1]) {
                if ("fps" == param_name_vec[2]) {
                    fps_or_time_flag = 1;
                    trigger_fps[camera_name] = parameter.as_double();
                    #if 1
                        RCLCPP_INFO_STREAM(
                            get_logger(),
                            "得到了为相机 " << camera_name << " 准备的 trigger.fps 参数" 
                        );
                    #endif
                }
            }
            if (!fps_or_time_flag) continue;

            // 处理exposure.time和trigger.fps之间的相关逻辑
            // 要求 time / 1000. < 1000. / fps
            if (0. != exposure_time[camera_name]) {
                if (0. != trigger_fps[camera_name]) {
                    if (1000. / trigger_fps[camera_name] < exposure_time[camera_name] / 1000.) {
                        if ("time" == param_name_vec[2]) {
                            RCLCPP_ERROR_STREAM(
                                get_logger(),
                                "相机名为 " << camera_name << 
                                " 的相机收到将曝光时间设置为 " << exposure_time[camera_name] << 
                                "(us) 的请求，但这与希望的取图帧率 " << trigger_fps[camera_name] << "(fps) 不符"
                            );
                            result.reason = "should new_exposure_time < 1000./fps (ms)";
                            result.successful = false;
                        } else {
                            RCLCPP_ERROR_STREAM(
                                get_logger(),
                                "相机名为 " << camera_name << 
                                " 的相机收到将取图帧率设置为 " << trigger_fps[camera_name] << 
                                "(fps) 的请求，但这与希望的曝光时间 " << exposure_time[camera_name] << "(us) 不符"
                            );
                            result.reason = "should new_exposure_time < 1000./fps (ms)";
                            result.successful = false;
                        }
                    }
                } else if ("trigger" == param_name_vec[1] && "fps" == param_name_vec[2]) {
                    RCLCPP_INFO(get_logger(), "最好先设置相机期望帧率再设置相机曝光时间");
                }
            }
        } else if (2 ==  param_name_vec.size()) {
            std::string camera_name =  param_name_vec[0];
            if ("record" == param_name_vec[1]) {
                // 说明是该record参数的初始化
                if (!record_init_flag_[camera_name]) {
                    record_init_flag_[camera_name] = true;
                    RCLCPP_INFO_STREAM(
                        get_logger(), "相机 " << camera_name << " 的录制使能参数初始化"
                    );
                    continue;
                }

                bool record_flag = parameter.as_bool();
                if (record_flag == video_record_need_flag_[camera_name]) {
                    std::string word = record_flag?"开始录制了":"结束录制了";
                    RCLCPP_ERROR_STREAM(
                        get_logger(), "相机 " << camera_name << " 已经"+word
                    );
                    word = record_flag?"true":"false";
                    result.reason = camera_name+".record already "+word;
                    result.successful = false;
                } else {
                    std::string word = record_flag?"开始录制":"结束录制";
                    RCLCPP_INFO_STREAM(
                        get_logger(), "相机 " << camera_name << " 将会"+word
                    );
                }
            }
        }
    }
    
    return result;
}

/**
 * @brief 查看具体参数设置事件有那些操作（忽略服务质量重载）
 * @return 返回服务质量的之外是否有新参数、参数改变、参数删除
 */
bool CameraNode::on_parameter_event_(rcl_interfaces::msg::ParameterEvent::UniquePtr & event, rclcpp::Logger logger)
{
    std::stringstream ss;
    // 忽略服务质量重载
    event->new_parameters.erase(
        std::remove_if(
            event->new_parameters.begin(),
            event->new_parameters.end(),
            [](const auto & item) {
                const char * param_override_prefix = "qos_overrides.";
                return std::strncmp(
                    item.name.c_str(), param_override_prefix, sizeof(param_override_prefix)-1) == 0u;
            }),
        event->new_parameters.end());
    // 查看除服务质量的之外是否有新参数、参数改变、参数删除
    if (!event->new_parameters.size() && !event->changed_parameters.size() &&
    !event->deleted_parameters.size()) return false;
    #if 0
        ss << "\nParameter event:\n new parameters:";
        for (auto & new_parameter : event->new_parameters) {
            ss << "\n  " << new_parameter.name;
        }
        ss << "\n changed parameters:";
        for (auto & changed_parameter : event->changed_parameters) {
            ss << "\n  " << changed_parameter.name;
        }
        ss << "\n deleted parameters:";
        for (auto & deleted_parameter : event->deleted_parameters) {
            ss << "\n  " << deleted_parameter.name;
        }
        ss << "\n";
        RCLCPP_INFO(logger, "%s", ss.str().c_str());
    #endif
    return true;
}

/**
 * @brief 负责处理参数的新建、修改、删除，将参数设置到成员变量上
 */
void CameraNode::ParamEventCallback_(rcl_interfaces::msg::ParameterEvent::UniquePtr event)
{
    if (!on_parameter_event_(event, get_logger())) return;
    //TODO: 这里应该也需要忽略服务质量重载
    for (auto & new_parameter : event->new_parameters) {
        std::vector<std::string> names = param_name_split_(new_parameter.name);
        if (3 == names.size()){
            std::string cname = names[0];
            if ("exposure" == names[1]) {
                if ("time" == names[2]) {
                    cam_[cname].exposure.time = new_parameter.value.double_value;
                    CameraSetExposureTime(cam_[cname].handle, cam_[cname].exposure.time);
                    CameraGetExposureTime(cam_[cname].handle, &cam_[cname].exposure.time);
                    RCLCPP_INFO_STREAM(
                        get_logger(),
                        "希望设置 " << cname << " 相机的曝光时间为 " << new_parameter.value.double_value << "(us) ，实际设置为 " << cam_[cname].exposure.time << "(us)"
                    );
                }
            } else if ("trigger" == names[1]) {
                if ("fps" == names[2]) {
                    cam_[cname].trigger.fps = new_parameter.value.double_value;
                    cam_task_timer_[cname] = create_wall_timer(
                        std::chrono::milliseconds(static_cast<int>(1000/cam_[cname].trigger.fps)), std::function([this, cname]{cam_task_(cname);}));
                    cam_[cname].trigger.fps = 1000. / static_cast<int>(1000/cam_[cname].trigger.fps);
                    RCLCPP_INFO_STREAM(
                        get_logger(),
                        "希望设置 " << cname << " 相机的定时器对应的取图频率为 " << new_parameter.value.double_value << "(fps) ，实际设置为 " << cam_[cname].trigger.fps << "(fps)"
                    );
                }
            }
        } else if (2 == names.size()) {
            std::string cname = names[0];
            if ("record" == names[1]) {
                video_record_need_flag_[cname] = new_parameter.value.bool_value;
                std::string word = video_record_need_flag_[cname]?"希望录制":"不希望录制";
                RCLCPP_INFO_STREAM(
                    get_logger(), "初始化时相机 " << cname << " "+word
                );
                if (!video_record_need_flag_[cname]) {
                    RCLCPP_INFO_STREAM(
                        get_logger(), "相机 " << cname << " 在初始化没有配置录制"
                    );
                } else {
                    //TODO: 放在这个位置合适吗？需要去检查核实
                    int fourcc = cv::VideoWriter::fourcc('M','J','P','G');   // 设置编码格式为motion jpeg(MJPEG)
                    auto size = cv::Size(cam_[cname].size.width, cam_[cname].size.height);
                    video_recorder_[cname].open("Camera/record_video/"+cname+get_local_time()+".avi", fourcc, cam_[cname].trigger.fps, size);
                    RCLCPP_INFO_STREAM(
                        get_logger(), "相机 " << cname << " 在 "+get_local_time()+" 尝试开始一次录制"
                    );
                }
            }
        }
        
    }

    for (auto & changed_parameter : event->changed_parameters) {
        std::vector<std::string> names = param_name_split_(changed_parameter.name);
        if (3 == names.size()){
            std::string cname = names[0];
            if ("exposure" == names[1]) {
                if ("time" == names[2]) {
                    cam_[cname].exposure.time = changed_parameter.value.double_value;
                    CameraSetExposureTime(cam_[cname].handle, cam_[cname].exposure.time);
                    CameraGetExposureTime(cam_[cname].handle, &cam_[cname].exposure.time);
                    RCLCPP_INFO_STREAM(
                        get_logger(),
                        "希望设置 " << cname << " 相机的曝光时间为 " << changed_parameter.value.double_value << "(us) ，实际设置为 " << cam_[cname].exposure.time << "(us)"
                    );
                }
            } else if ("trigger" == names[1]) {
                if ("fps" == names[2]) {
                    cam_[cname].trigger.fps = changed_parameter.value.double_value;
                    cam_task_timer_[cname]->cancel();
                    cam_task_timer_[cname] = create_wall_timer(
                        std::chrono::milliseconds(static_cast<int>(1000/cam_[cname].trigger.fps)), std::function([this, cname]{cam_task_(cname);}));
                    cam_[cname].trigger.fps = 1000. / static_cast<int>(1000/cam_[cname].trigger.fps);
                    RCLCPP_INFO_STREAM(
                        get_logger(),
                        "希望设置 " << cname << " 相机的定时器对应的取图频率为 " << changed_parameter.value.double_value << "(fps) ，实际设置为 " << cam_[cname].trigger.fps << "(fps)"
                    );
                }
            }
        } else if (2 == names.size()) {
            std::string cname = names[0];
            if ("record" == names[1]) {
                video_record_need_flag_[cname] = changed_parameter.value.bool_value;
                std::string word = video_record_need_flag_[cname]?"希望录制":"不希望录制";
                if (!video_record_need_flag_[cname]) {
                    video_recorder_[cname].release();
                    RCLCPP_INFO_STREAM(
                        get_logger(), "相机 " << cname << " 在 "+get_local_time()+" 结束一次录制"
                    );
                } else {
                    //TODO: 放在这个位置合适吗？需要去检查核实
                    int fourcc = cv::VideoWriter::fourcc('M','J','P','G');   // 设置编码格式为motion jpeg(MJPEG)
                    auto size = cv::Size(cam_[cname].size.width, cam_[cname].size.height);
                    video_recorder_[cname].open("Camera/record_video/"+cname+get_local_time()+".avi", fourcc, cam_[cname].trigger.fps, size);
                    RCLCPP_INFO_STREAM(
                        get_logger(), "相机 " << cname << " 在 "+get_local_time()+" 尝试开始一次录制"
                    );
                }
            }
        }
    }
}

void CameraNode::image_publish_(const std::string & camera_name)
{
    cv::Mat image = GetImage(camera_name);
    frame_[camera_name] = image;    //TODO: 这是为了录制存了一个成员变量，应该会有更好的写法
    if (!image.empty()) {
        sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
        msg->header.frame_id = camera_name;
        msg->header.stamp = this->now();
        msg->height = image.rows;
        msg->width = image.cols;
        msg->encoding = mat_type2encoding(image.type());
        msg->is_bigendian = false;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
        msg->data.assign(image.datastart, image.dataend);
        image_pub_[camera_name]->publish(std::move(msg));
    }
}

std::string CameraNode::get_local_time()
{
    std::time_t t = std::time(0);
    std::tm * time_now = std::localtime(&t);
    std::ostringstream oss;
    oss << time_now->tm_year+1900 << "-" << time_now->tm_mon+1  << "-"
              << time_now->tm_mday << "-" << time_now->tm_hour << ":"
              << time_now->tm_min  << ":" << time_now->tm_sec;
    std::string result_time = oss.str();
    return result_time;
}

void CameraNode::video_record_(const std::string & camera_name)
{
    if(!video_recorder_[camera_name].isOpened()) {
        #if 1
            RCLCPP_INFO(get_logger(), "图片录制初始化");
        #endif
        int fourcc = cv::VideoWriter::fourcc('M','J','P','G');   // 设置编码格式为motion jpeg(MJPEG)
        auto size = cv::Size(cam_[camera_name].size.width, cam_[camera_name].size.height);
        video_recorder_[camera_name].open("Camera/record_video/"+camera_name+get_local_time()+".avi", fourcc, cam_[camera_name].trigger.fps, size);
    } else {
        #if 0
            RCLCPP_INFO(get_logger(), "图片录制");
        #endif
        video_recorder_[camera_name].write(frame_[camera_name]);
        #if 1
            cv::imshow("录制中", frame_[camera_name]);
            cv::waitKey(1);
        #endif
    }
}
}
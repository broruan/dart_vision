#include <communicate_2025/link.hpp>

RMLink::RMLink(
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config,
    std::string node_name
):
    Node(node_name),

    device_config_(std::move(*device_config)),
    owned_ctx_(nullptr),
    stm32_serial_(nullptr) {
    // 初始化IO驱动
    try {
        owned_ctx_ = std::make_unique<IoContext>(2);
        stm32_serial_ = std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_);
    } catch (const std::exception& e) {
        std::cerr << "初始化IO驱动失败" << e.what() << std::endl;
        exit(-5);
    }
    this->debug_ = this->declare_parameter("debug", false);
    this->serial_enable_ = this->declare_parameter("serial_enable", true);
    this->serial_name_ = this->declare_parameter("serial_name", "/dev/ttyACM0");
    this->enemy_team_color_ = this->declare_parameter("enemy_team_color", 1);
    this->robot_type_ = this->declare_parameter("robot_type_", 0);
    stm32_serial_->init_port(serial_name_, device_config_);

    this->Init_Pub_and_Sub();

    if (!this->serial_enable_) {
        std::cout << "以调试模式启动" << std::endl;
    } else {
        this->reopen_count_ = 0;
        // 初始化串口
        try {
            stm32_serial_->port()->open();
            if (stm32_serial_->port()->is_open()) {
                std::cout << "!打开串口成功!" << std::endl;
            }
            // 串口打开失败，尝试重连，最多5次
        } catch (const std::exception& e) {
            std::cerr << "!打开串口失败!" << e.what() << std::endl;
            while (ReopenPort() == false) {
                if (reopen_count_++ > 5) {
                    std::cerr << "!重开串口失败!" << e.what() << std::endl;
                    exit(-1);
                }
            }
        }
    }
}

// 尝试重连串口
bool RMLink::ReopenPort() {
    // 重连警告
    std::cerr << "Attempting to reopen port " << serial_name_ << std::endl;
    // 尝试重启port
    try {
        if (stm32_serial_->port()->is_open()) {
            stm32_serial_->port()->close();
        }
        stm32_serial_->port()->open();
        std::cout << "Reopen port success" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Reopen port " << serial_name_ << " unsuccess" << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}

// 从下位机接收消息
void RMLink::Recv() {
    // 调试模式
    if (!this->serial_enable_) {
        this->PublishGyroDefault(this->Debug_Pub_[GYRO_DEFAULT]);
        this->PublishAutoaimDefault(this->Debug_Pub_[AUTOAIM_DEFAULT]);
        this->PublishDetectInfoDefault(this->Debug_Pub_[DETECT_INFO_DEFAULT]);
        return;
    }

    // 接收数据
    try {
        std::vector<uint8_t> data(sizeof(Message));
        stm32_serial_->port()->receive(data);
        this->datarecv = fromVector(data);
    } catch (const std::exception& e) {
        std::cerr << "(>_<)!消息接收失败!" << e.what() << std::endl;
        exit(-3);
    }

    // 检验包头包尾，数据是否是下位机发上位机
    // 包头包尾为's'和'e'，类型在0xB0到0xC0之间
    // 如果不符合要求，打印错误信息并退出
    // 如果符合要求，将类型减去0xB0，并调用对应的发布函数

    // TODO: 数据范围在这里是硬编码的，改为 config 中定义

    if (this->datarecv.start != 's' || this->datarecv.end != 'e' || this->datarecv.type < 0xB0
        || this->datarecv.type > 0xC0)
    {
        std::cerr << "(>_<)!数据包校验失败!" << std::endl;
        exit(-4);
    } else {
        this->datarecv.type -= 0xB0;

        // 发布信息话题
        // TODO: 函数指针数组可能不安全，建议使用 unordered_map
        try {
            // TODO： 调试模式开一个话题输出16进制数据, 或者直接打印
            (this->*Func[this->datarecv.type]
            )(this->datarecv.buffer, this->Pub_[this->datarecv.type]);
        } catch (const std::exception& e) {
            std::cerr << "(>_<)!未找到对应发布者!" << e.what() << std::endl;
            exit(-4);
        }
    }
}

/**
 * @brief 向下位机发送数据
 * @tparam T 数据类型
 * @param type 消息类型
 * @param buffer 数据
 */
template<typename T>
void RMLink::Send(uint8_t type, T* buffer) {
    this->datasend.start = 's';
    this->datasend.end = 'e';
    this->datasend.type = type;
    memcpy(this->datasend.buffer, (uint8_t*)buffer, sizeof(T));
    auto data = toVector(this->datasend);
    try {
        // TODO： 调试模式开一个话题输出16进制数据
        // TODO： 调试模式开一个话题, 输入16进制数据，直接发送对应值
        stm32_serial_->port()->send(data);
    } catch (const std::exception& e) {
        std::cerr << "(>_<)!消息发送失败!" << e.what() << std::endl;
        exit(-2);
    }
}

// 初始化发布者和订阅者
void RMLink::Init_Pub_and_Sub() {
    // 创建发布者

    // 无串口模式
    if (!this->serial_enable_) {
        this->Debug_Pub_[GYRO_DEFAULT] = this->create_publisher<sensor_msgs::msg::JointState>(
            "/communicate/debug/gyro",
            rclcpp::SystemDefaultsQoS()
        );
        this->Debug_Pub_[AUTOAIM_DEFAULT] = this->create_publisher<communicate_2025::msg::Autoaim>(
            "/communicate/debug/autoaim",
            rclcpp::SystemDefaultsQoS()
        );
        this->Debug_Pub_[DETECT_INFO_DEFAULT] = this->create_publisher<communicate_2025::msg::SerialInfo>(
            "/detect_info",
            rclcpp::SystemDefaultsQoS()
        );
    }
    // 串口模式
    else
    {
        // 自瞄话题
        if (this->robot_type_ == SENTINEL || this->robot_type_ == HERO
            || this->robot_type_ == INFANTRY) {
            this->Pub_[AUTOAIM] = this->create_publisher<communicate_2025::msg::Autoaim>(
                "/communicate/autoaim",
                rclcpp::SystemDefaultsQoS()
            );
        }

        // 烧饼话题
        if (this->robot_type_ == SENTINEL) {
            this->Pub_[ROBOT_POSITION] = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "/communicate/position/robot",
                rclcpp::SystemDefaultsQoS()
            );
            this->Pub_[ROBOT_EX_POSITION] =
                this->create_publisher<std_msgs::msg::Float32MultiArray>(
                    "/communicate/position/exrobot",
                    rclcpp::SystemDefaultsQoS()
                );
            this->Pub_[ENEMY_ROBOT_POSITION] =
                this->create_publisher<std_msgs::msg::Float32MultiArray>(
                    "/communicate/position/enemyrobot",
                    rclcpp::SystemDefaultsQoS()
                );
            this->Pub_[ENEMY_EX_ROBOT_POSITION] =
                this->create_publisher<std_msgs::msg::Float32MultiArray>(
                    "/communicate/position/enemyexrobot",
                    rclcpp::SystemDefaultsQoS()
                );
            this->Pub_[RED_ROBOT_HP] = this->create_publisher<std_msgs::msg::Int32MultiArray>(
                "/communicate/hp/redrobot",
                rclcpp::SystemDefaultsQoS()
            );
            this->Pub_[BLUE_ROBOT_HP] = this->create_publisher<std_msgs::msg::Int32MultiArray>(
                "/communicate/hp/bluerobot",
                rclcpp::SystemDefaultsQoS()
            );
            this->Pub_[BUILDING_HP] = this->create_publisher<std_msgs::msg::Int32MultiArray>(
                "/communicate/hp/building",
                rclcpp::SystemDefaultsQoS()
            );
            this->Pub_[GAME_INFO] = this->create_publisher<std_msgs::msg::Int32MultiArray>(
                "/communicate/gameinfo",
                rclcpp::SystemDefaultsQoS()
            );
            this->Pub_[SHOOT_STATUS] = this->create_publisher<std_msgs::msg::Int32MultiArray>(
                "/communicate/shootstatus",
                rclcpp::SystemDefaultsQoS()
            );
            this->Pub_[COMMAND] = this->create_publisher<communicate_2025::msg::Command>(
                "/communicate/command",
                rclcpp::SystemDefaultsQoS()
            );
            this->Pub_[HITTED] = this->create_publisher<std_msgs::msg::Int32MultiArray>(
                "/communicate/hitted",
                rclcpp::SystemDefaultsQoS()
            );
        }

        // 英雄话题
        if (this->robot_type_ == HERO) {
            this->Pub_[SHOOT_STATUS] = this->create_publisher<std_msgs::msg::Int32MultiArray>(
                "/communicate/shootstatus",
                rclcpp::SystemDefaultsQoS()
            );
        }

        // TODO: 明年没人做就砍了好了
        // 工程话题
        if (this->robot_type_ == ENGINEER) {
            this->Pub_[ENGINEER_ARM] = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "/communicate/engineerarm",
                rclcpp::SystemDefaultsQoS()
            );
            this->Pub_[ENGINEER_INTERACTION] =
                this->create_publisher<std_msgs::msg::Int32MultiArray>(
                    "/communicate/engineerinteraction",
                    rclcpp::SystemDefaultsQoS()
                );
        }
    }

    // 发布者函数指针数组
    Func[AUTOAIM] = &RMLink::PublishAutoaim;

    if (this->robot_type_ == SENTINEL) {
        Func[ROBOT_POSITION] = &RMLink::PublishRobotPosition;
        Func[ROBOT_EX_POSITION] = &RMLink::PublishExRobotPosition;
        Func[ENEMY_ROBOT_POSITION] = &RMLink::PublishEnemyRobotPosition;
        Func[ENEMY_EX_ROBOT_POSITION] = &RMLink::PublishEnemyExRobotPosition;
        Func[RED_ROBOT_HP] = &RMLink::PublishRedRobotHP;
        Func[BLUE_ROBOT_HP] = &RMLink::PublishBlueRobotHP;
        Func[BUILDING_HP] = &RMLink::PublishBuildingHP;
        Func[GAME_INFO] = &RMLink::PublishGameInfo;
        Func[COMMAND] = &RMLink::PublishCommand;
        Func[HITTED] = &RMLink::PublishHitted;
        Func[SHOOT_STATUS] = &RMLink::PublishShootStatus;
    }

    if (this->robot_type_ == HERO) {
        Func[SHOOT_STATUS] = &RMLink::PublishShootStatus;
    }

    if (this->robot_type_ == ENGINEER) {
        Func[ENGINEER_ARM] = &RMLink::PublishEngineerArm;
        Func[ENGINEER_INTERACTION] = &RMLink::PublishEngineerInteraction;
    }

    //TODO: 话题名称写进 config 文件中
    // 创建订阅者
    // 增加飞镖类型
    if (this->robot_type_ == INFANTRY || this->robot_type_ == HERO || this->robot_type_ == SENTINEL || this->robot_type_ == DART)
    {
        this->Autoaim_sub = this->create_subscription<communicate_2025::msg::SerialInfo>(
            "/shoot_info",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RMLink::GimbalCB, this, std::placeholders::_1)
        );
        this->AutoaimWithDist_sub = this->create_subscription<communicate_2025::msg::SerialInfo>(
            "/detect_info",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RMLink::GimbalWithVelCB, this, std::placeholders::_1)
        );
    }

    if (this->robot_type_ == SENTINEL) {
        this->Chassis_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "sentry/cmd_vel",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RMLink::ChassisCB, this, std::placeholders::_1)
        );

        this->Interaction_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/behaviortree/interaction",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RMLink::InteractionCB, this, std::placeholders::_1)
        );

        this->Moudle_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/behaviortree/moudle",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RMLink::MoudleCB, this, std::placeholders::_1)
        );
    }

    if (this->robot_type_ == ENGINEER) {
        this->EngineerArm_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/engineer/arm_control",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RMLink::EngineerArmCB, this, std::placeholders::_1)
        );

        this->EngineerInteraction_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/engineer/interaction_control",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&RMLink::EngineerInteractionCB, this, std::placeholders::_1)
        );
    }
}

/*
调试通信部分
*/

// 发布默认tf树测试信息话题
void RMLink::PublishGyroDefault(rclcpp::PublisherBase::SharedPtr pub) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.position.push_back(0);
    msg.position.push_back(0);
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(pub);
    pub_completed->publish(msg);
}

// 发布自瞄测试信息话题
void RMLink::PublishAutoaimDefault(rclcpp::PublisherBase::SharedPtr pub) {
    communicate_2025::msg::Autoaim msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "shooter";
    msg.high_gimbal_yaw = 0;
    msg.pitch = 0;
    msg.enemy_team_color = 0;
    msg.mode = 0;
    msg.rune_flag = 0;
    msg.low_gimbal_yaw = 0;
    rclcpp::Publisher<communicate_2025::msg::Autoaim>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<communicate_2025::msg::Autoaim>>(pub);
    pub_completed->publish(msg);
}

// 发布检测信息假数据话题
void RMLink::PublishDetectInfoDefault(rclcpp::PublisherBase::SharedPtr pub) {
    communicate_2025::msg::SerialInfo msg;
    msg.yaw = 0;
    msg.pitch = 0;
    msg.s = 0;
    msg.is_find.data = 0;
    rclcpp::Publisher<communicate_2025::msg::SerialInfo>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<communicate_2025::msg::SerialInfo>>(pub);
    pub_completed->publish(msg);
}

/*
串口通信部分
*/

// 发布自瞄云台信息话题
void RMLink::PublishAutoaim(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    Autoaim* data = (Autoaim*)buffer;
    communicate_2025::msg::Autoaim msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "shooter";
    msg.high_gimbal_yaw = data->high_gimbal_yaw;
    msg.pitch = data->pitch;
    msg.enemy_team_color = this->enemy_team_color_; // 使用config文件中的敌方颜色
    msg.mode = data->mode;
    msg.rune_flag = data->rune_flag;
    msg.low_gimbal_yaw = data->low_gimbal_yaw;

    if (msg.mode == 2 && this->robot_type_ == HERO) {
        std::cerr << "PublishAutoaim is receiving message in ["
                  << (this->debug_ ? "debug" : Robot_Type_String[this->robot_type_])
                  << "] mode, expected in HERO mode. " << std::endl;
    }
    rclcpp::Publisher<communicate_2025::msg::Autoaim>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<communicate_2025::msg::Autoaim>>(pub);
    pub_completed->publish(msg);
}

// 订阅自瞄控制话题回调函数
void RMLink::GimbalCB(const communicate_2025::msg::SerialInfo::SharedPtr msg) {
    if (!this->serial_enable_) {
        return;
    }
    GimbalControl tmp;
    tmp.find_bools = msg->is_find.data;
    tmp.yaw = msg->yaw;
    tmp.pitch = msg->pitch;
    RMLink::Send(0xA0, &tmp);
}

// 订阅飞镖的自瞄控制话题回调函数
void RMLink::GimbalWithVelCB(const communicate_2025::msg::SerialInfo::SharedPtr msg) {
    if (!this->serial_enable_) {
        return;
    }
    GimbalControlWithVel tmp;
    tmp.find_bools = msg->is_find.data;
    tmp.yaw = msg->yaw;
    tmp.pitch = msg->pitch;
    tmp.s = msg->s;
    RMLink::Send(0xA6, &tmp);
}

/**
 * @brief 订阅底盘速度控制话题回调函数
 * @param msg Twist
 */
void RMLink::ChassisCB(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!this->serial_enable_) {
        return;
    }
    ChassisControl tmp;
    tmp.x_speed = msg->linear.x;
    tmp.y_speed = msg->linear.y;
    tmp.yaw = msg->angular.x;
    RMLink::Send(0xA1, &tmp);
}

/**
 * @brief 订阅比赛交互控制话题回调函数
 * @param msg Int32MultiArray
 * @details 仅烧饼使用
 */
void RMLink::InteractionCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (!this->serial_enable_) {
        return;
    }
    if (this->robot_type_ != SENTINEL) {
        std::cerr << "InteractionCB is receiving message in ["
                  << (this->debug_ ? "debug" : Robot_Type_String[this->robot_type_])
                  << "] mode, expected in SENTINEL mode. Received type: " << msg->data[0]
                  << ", content: " << msg->data[1] << std::endl;
    }
    InteractionControl tmp;
    tmp.type = msg->data[0];
    tmp.content = msg->data[1];
    RMLink::Send(0xA2, &tmp);
}

/**
 * @brief 订阅模块控制话题回调函数
 * @param msg Int32MultiArray
 * @details 仅烧饼使用
 */
void RMLink::MoudleCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (!this->serial_enable_) {
        return;
    }
    if (this->robot_type_ != SENTINEL) {
        std::cerr << "MoudleCB is receiving message in ["
                  << (this->debug_ ? "debug" : Robot_Type_String[this->robot_type_])
                  << "] mode, expected in SENTINEL mode. Received type: " << msg->data[0]
                  << ", content: " << msg->data[1] << std::endl;
    }
    MoudleControl tmp;
    tmp.type = msg->data[0];
    tmp.content = msg->data[1];
    RMLink::Send(0xA3, &tmp);
}

/**
 * @brief 订阅工程机械臂控制话题回调函数
 * @param msg Float32MultiArray
 * @details 仅工程使用
 */
void RMLink::EngineerArmCB(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (!this->serial_enable_) {
        return;
    }
    if (this->robot_type_ != ENGINEER) {
        std::cerr << "EngineerArmCB is receiving message in ["
                  << (this->debug_ ? "debug" : Robot_Type_String[this->robot_type_])
                  << "] mode, expected in ENGINEER mode. " << std::endl;
    }
    EngineerArmControl tmp;
    tmp.h0 = msg->data[0];
    tmp.x1 = msg->data[1];
    tmp.q2 = msg->data[2];
    tmp.q3 = msg->data[3];
    tmp.q4 = msg->data[4];
    tmp.q5 = msg->data[5];
    tmp.q6 = msg->data[6];

    RMLink::Send(0xA4, &tmp);
}
/**
 * @brief 订阅工程交互控制话题回调函数
 * @param msg Int32MultiArray
 * @details 仅工程使用
 */
void RMLink::EngineerInteractionCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (!this->serial_enable_) {
        return;
    }
    if (this->robot_type_ != ENGINEER) {
        std::cerr << "EngineerInteractionCB is receiving message in ["
                  << (this->debug_ ? "debug" : Robot_Type_String[this->robot_type_])
                  << "] mode, expected in ENGINEER mode. Received type: " << msg->data[0]
                  << ", content: " << msg->data[1] << std::endl;
    }
    EngineerInteractionControl tmp;
    tmp.type = msg->data[0];
    tmp.content = msg->data[1];
    RMLink::Send(0xA5, &tmp);
}

/*
裁判系统部分
*/

// 发布我方机器人位置信息话题1
void RMLink::PublishRobotPosition(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    RobotPosition* data = (RobotPosition*)buffer;
    std_msgs::msg::Float32MultiArray msg;
    msg.data.push_back(data->infantry_3_x);
    msg.data.push_back(data->infantry_3_y);
    msg.data.push_back(data->infantry_4_x);
    msg.data.push_back(data->infantry_4_y);
    msg.data.push_back(data->infantry_5_x);
    msg.data.push_back(data->infantry_5_y);
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布我方机器人位置信息话题2
void RMLink::PublishExRobotPosition(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    ExRobotPosition* data = (ExRobotPosition*)buffer;
    std_msgs::msg::Float32MultiArray msg;
    msg.data.push_back(data->hero_x);
    msg.data.push_back(data->hero_y);
    msg.data.push_back(data->engineer_x);
    msg.data.push_back(data->engineer_y);
    msg.data.push_back(data->sentinel_x);
    msg.data.push_back(data->sentinel_x);
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布敌方机器人位置信息话题1
void RMLink::PublishEnemyRobotPosition(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    EnemyRobotPosition* data = (EnemyRobotPosition*)buffer;
    std_msgs::msg::Float32MultiArray msg;
    msg.data.push_back(data->enemy_infantry_3_x);
    msg.data.push_back(data->enemy_infantry_3_y);
    msg.data.push_back(data->enemy_infantry_4_x);
    msg.data.push_back(data->enemy_infantry_4_y);
    msg.data.push_back(data->enemy_infantry_5_x);
    msg.data.push_back(data->enemy_infantry_5_y);
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布敌方机器人位置信息话题2
void RMLink::PublishEnemyExRobotPosition(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    ExEnemyRobotPosition* data = (ExEnemyRobotPosition*)buffer;
    std_msgs::msg::Float32MultiArray msg;
    msg.data.push_back(data->enemy_hero_x);
    msg.data.push_back(data->enemy_hero_y);
    msg.data.push_back(data->enemy_engineer_x);
    msg.data.push_back(data->enemy_engineer_y);
    msg.data.push_back(data->enemy_sentinel_x);
    msg.data.push_back(data->enemy_sentinel_y);
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布红方机器人血量信息话题
void RMLink::PublishRedRobotHP(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    RedRobotHP* data = (RedRobotHP*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->red_1_robot_HP);
    msg.data.push_back(data->red_2_robot_HP);
    msg.data.push_back(data->red_3_robot_HP);
    msg.data.push_back(data->red_4_robot_HP);
    msg.data.push_back(data->red_5_robot_HP);
    msg.data.push_back(data->red_7_robot_HP);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布蓝方机器人血量信息话题
void RMLink::PublishBlueRobotHP(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    BlueRobotHP* data = (BlueRobotHP*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->blue_1_robot_HP);
    msg.data.push_back(data->blue_2_robot_HP);
    msg.data.push_back(data->blue_3_robot_HP);
    msg.data.push_back(data->blue_4_robot_HP);
    msg.data.push_back(data->blue_5_robot_HP);
    msg.data.push_back(data->blue_7_robot_HP);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布建筑血量信息话题
void RMLink::PublishBuildingHP(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    BuildingHP* data = (BuildingHP*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->red_outpost_HP);
    msg.data.push_back(data->red_base_HP);
    msg.data.push_back(data->blue_outpost_HP);
    msg.data.push_back(data->blue_base_HP);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布比赛信息话题
void RMLink::PublishGameInfo(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    GameInfo* data = (GameInfo*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(this->enemy_team_color_); // 使用config文件中的敌方颜色
    msg.data.push_back(data->game_progress);
    msg.data.push_back(data->stage_remain_time);
    msg.data.push_back(data->remaining_gold_coin);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布发射状态量信息话题
void RMLink::PublishShootStatus(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    ShootStatus* data = (ShootStatus*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->projectile_allowance_17mm);
    msg.data.push_back(data->projectile_allowance_42mm);
    msg.data.push_back(data->real_heat);
    msg.data.push_back(data->launching_frequency);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布操作反馈信息话题
void RMLink::PublishCommand(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    Command* data = (Command*)buffer;
    communicate_2025::msg::Command msg;
    msg.target_position_x = data->target_position_x;
    msg.target_position_y = data->target_position_y;
    msg.cmd_keyboard = data->cmd_keyboard;
    msg.target_robot_id = data->target_robot_id;
    rclcpp::Publisher<communicate_2025::msg::Command>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<communicate_2025::msg::Command>>(pub);
    pub_completed->publish(msg);
}

// 发布受击反馈信息话题
void RMLink::PublishHitted(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    Hitted* data = (Hitted*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->hitted);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布工程机械臂信息话题
void RMLink::PublishEngineerArm(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    EngineerArm* data = (EngineerArm*)buffer;
    std_msgs::msg::Float32MultiArray msg;
    msg.data.push_back(data->h0);
    msg.data.push_back(data->x1);
    msg.data.push_back(data->q2);
    msg.data.push_back(data->q3);
    msg.data.push_back(data->q4);
    msg.data.push_back(data->q5);
    msg.data.push_back(data->q6);
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub);
    pub_completed->publish(msg);
}

// 发布工程交互信息话题
void RMLink::PublishEngineerInteraction(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub) {
    EngineerInteraction* data = (EngineerInteraction*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->type);
    msg.data.push_back(data->content);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed =
        std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}
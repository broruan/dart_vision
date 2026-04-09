#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "communicate_2025/msg/autoaim.hpp"     // IWYU pragma: keep
#include "communicate_2025/msg/command.hpp"     // IWYU pragma: keep
#include "communicate_2025/msg/serial_info.hpp" // IWYU pragma: keep

#include "protocol.hpp"

enum Robot_Type { SENTINEL = 0, HERO, INFANTRY, ENGINEER };

class RMLink: public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param serial_name 串口路径
     * @param device_config 串口配置
     */
    RMLink(
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config,
        std::string node_name = "communicate_node"
    );

    /**
     * @brief 向下位机发送数据
     * @tparam T 数据类型
     * @param type 消息类型
     * @param buffer 数据
     */
    template<typename T>
    void Send(uint8_t type, T* buffer);

    /**
     * @brief 从下位机接收消息
     */
    void Recv();

    /**
     * @brief 初始化发布者和订阅者
     */
    void Init_Pub_and_Sub();

    /**
     * @brief 重开串口
     */
    bool ReopenPort();

    /*
    调试通信部分
    */

    /**
     * @brief 发布默认tf树假信息话题
     * @note 仅在debug模式下使用
     * @param pub 发布者
     */
    void PublishGyroDefault(rclcpp::PublisherBase::SharedPtr pub);

    /**
     * @brief 发布自瞄假信息话题
     * @note 仅在debug模式下使用
     * @param pub 发布者
     */
    void PublishAutoaimDefault(rclcpp::PublisherBase::SharedPtr pub);

    /*
    串口通信部分
    */

    /**
     * @brief 发布自瞄云台话题
     * @param pub 发布者,发布云台控制
     * @param buffer 发送数据
     */
    void PublishAutoaim(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
     * @brief 订阅云台控制话题回调函数
     */
    void GimbalCB(const communicate_2025::msg::SerialInfo::SharedPtr msg);

    /**
     * @brief 订阅底盘速度控制话题回调函数
     */
    void ChassisCB(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief 订阅比赛交互控制话题回调函数
     */
    void InteractionCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    /**
     * @brief 订阅车体模块控制话题回调函数
     */
    void MoudleCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    /**
     * @brief 订阅工程机械臂控制话题回调函数
     */
    void EngineerArmCB(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /**
     * @brief 订阅工程交互控制话题回调函数
     */
    void EngineerInteractionCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    /*
    裁判系统部分
    */

    /**
     * @brief 发布我方机器人位置信息话题(1 2 3 号机器人)
     * @param pub 发布者
     * @param buffer 发送数据
     */
    void PublishRobotPosition(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布我方机器人位置信息话题(4 5 7 号机器人)
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishExRobotPosition(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布敌方机器人位置信息话题(1 2 3 号机器人)
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishEnemyRobotPosition(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布敌方机器人位置信息话题(4 5 7 号机器人)
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishEnemyExRobotPosition(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布红方机器人血量信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishRedRobotHP(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布蓝方机器人血量信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishBlueRobotHP(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布建筑血量信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishBuildingHP(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布比赛信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishGameInfo(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布操作反馈信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishCommand(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布受击反馈信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishHitted(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布发射状态量信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishShootStatus(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
    * @brief 发布机械臂控制信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishEngineerArm(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

    /**
     * @brief 发布工程交互控制信息话题
     * @param pub 发布者
     * @param buffer 发送数据
     */
    void PublishEngineerInteraction(uint8_t* buffer, rclcpp::PublisherBase::SharedPtr pub);

private:
    bool debug_;         // 是否开启调试模式
    bool serial_enable_; // 是否开启串口通信

    int robot_type_; // 兵种类型
    const char* Robot_Type_String[4] = { "SENTINEL", "HERO", "INFANTRY", "ENGINEER" };

    std::string serial_name_;                                // 串口路径，来自yaml文件
    drivers::serial_driver::SerialPortConfig device_config_; // 串口配置
    std::unique_ptr<IoContext> owned_ctx_;                   // 控制异步io线程数
    std::unique_ptr<drivers::serial_driver::SerialDriver> stm32_serial_; // 串口

    int enemy_team_color_; // 敌方颜色 0：红 1：蓝

    int reopen_count_; // 重连次数

    Message datarecv; // 接收数据
    Message datasend; // 发送数据

    /*
    调试通信部分
    */

    // 调试功能ID : 0-1
    enum FunctionID_Debug {
        GYRO_DEFAULT = 0, // tf树假信息
        AUTOAIM_DEFAULT,  // 自瞄假信息
    };
    rclcpp::PublisherBase::SharedPtr Debug_Pub_[2]; // 发布者

    // TODO : 加一个可选的直接发布 tf 树功能

    /*
    通信部分
    */

    // TODO: 函数指针数组可能不安全，建议使用 unordered_map
    // TODO: 话题数量硬编码，使用 unordered_map 可规避并提高可读性
    void (RMLink::*Func[15])(uint8_t*, rclcpp::PublisherBase::SharedPtr); // 消息发布函数指针数组
    rclcpp::PublisherBase::SharedPtr Pub_[15];                            // 发布者

    /*
    串口通信部分
    */

    // 串口回调
    rclcpp::Subscription<communicate_2025::msg::SerialInfo>::SharedPtr
        Autoaim_sub;                                                        // 自瞄控制订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Chassis_sub; // 底盘控制订阅者
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
        Interaction_sub; // 比赛交互控制订阅者
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
        Moudle_sub; // 车体模块控制订阅者
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
        EngineerArm_sub; // 工程机械臂控制订阅者
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
        EngineerInteraction_sub; // 工程交互控制订阅者

    // 串口通信功能ID : 0
    enum FunctionID_Serial {
        AUTOAIM = 0, // 自瞄云台控制
    };

    /*
    裁判系统部分
    */

    // 裁判系统通信功能ID : 1-13
    enum FunctionID_Judgement {
        ROBOT_POSITION = 1,      // 1 机器人位置1
        ROBOT_EX_POSITION,       // 2 机器人位置2
        ENEMY_ROBOT_POSITION,    // 3 敌方机器人位置1
        ENEMY_EX_ROBOT_POSITION, // 4 敌方机器人位置2
        RED_ROBOT_HP,            // 5 红方机器人血量
        BLUE_ROBOT_HP,           // 6 蓝方机器人血量
        BUILDING_HP,             // 7 建筑血量
        GAME_INFO,               // 8 比赛信息
        COMMAND,                 // 9 操作反馈
        HITTED,                  // A 受击反馈
        SHOOT_STATUS,            // B 发射状态
        ENGINEER_ARM,            // C 工程机械臂控制
        ENGINEER_INTERACTION,    // D 工程交互控制
    };
};
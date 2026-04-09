#

## 原位置:烧饼uplink

### tf树假信息

发布 串口 debug

sensor_msgs::msg::JointState

```cpp
stamp : now()
position : (0,0)
```

```cpp
    /**
    * @brief 发布默认tf树假信息话题
    * @param pub 发布者
    */
    void PublishGyroDefault(
    rclcpp::PublisherBase::SharedPtr pub
    );
```

### 陀螺仪信息

发布 串口 可能计划弃用 下位机发上位机

```cpp
typedef struct GyroFeedback_s // 陀螺仪信息反馈接收数据
{
    float yaw;   // 偏航角
    float pitch; // 俯仰角
} AutoaimFeedbackBuffer;
```

```cpp
    /**
     * @brief 发布陀螺仪信息话题
     * @param pub 发布者
     * @param buffer 发送数据
     */
    void PublishGyro(
        uint8_t* buffer,
        rclcpp::PublisherBase::SharedPtr pub
    );
```

### 我方机器人位置1

发布 裁判系统

```cpp
std_msgs::msg::Float32MultiArray
typedef struct RobotPositionFeedback_s // 我方机器人位置反馈接收数据(1 2 3 号机器人)
{
    float standard_1_x; // 己方 1 号英雄机器人位置 x 轴坐标
    float standard_1_y; // 己方 1 号英雄机器人位置 y 轴坐标
    float standard_2_x; // 己方 2 号工程机器人位置 x 轴坐标
    float standard_2_y; // 己方 2 号工程机器人位置 y 轴坐标
    float standard_3_x; // 己方 3 号步兵机器人位置 x 轴坐标
    float standard_3_y; // 己方 3 号步兵机器人位置 y 轴坐标
} RobotPositionFeedbackBuffer;
```

```cpp
    /**
    * @brief 发布我方机器人位置信息话题(1 2 3 号机器人)
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishRobotPosition(
        uint8_t* buffer,
        rclcpp::PublisherBase::SharedPtr pub
    );
```

### 我方机器人位置2

发布 裁判系统

```cpp
std_msgs::msg::Float32MultiArray
typedef struct ExRobotPositionFeedback_s // 我方机器人位置反馈接收数据(4 5 7 号机器人)
{
    float standard_4_x; // 己方 4 号步兵机器人位置 x 轴坐标
    float standard_4_y; // 己方 4 号步兵机器人位置 y 轴坐标
    float standard_5_x; // 己方 5 号步兵机器人位置 x 轴坐标
    float standard_5_y; // 己方 5 号步兵机器人位置 y 轴坐标
    float standard_7_x; // 己方 7 号哨兵机器人位置 x 轴坐标
    float standard_7_y; // 己方 7 号哨兵机器人位置 y 轴坐标
} ExRobotPositionFeedbackBuffer;
```

```cpp
    /**
    * @brief 发布我方机器人位置信息话题(4 5 7 号机器人)
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishExRobotPosition(
        uint8_t* buffer,
        rclcpp::PublisherBase::SharedPtr pub
    );
```

### 比赛信息

发布 裁判系统

```cpp
std_msgs::msg::Int32MultiArray
typedef struct GameFeedback_s // 比赛反馈接收数据
{
    int enemy_team_color;   // 敌方颜色 0：红 1：蓝
    int game_progress;      // 比赛阶段
    int game_time;          // 比赛时间
    int game_economy;       // 比赛经济
    int allowance_capacity; // 允许发弹量
    int left_purchase;      // 左发射机构状态
    int right_purchase;     // 右发射机构状态
} GameFeedbackBuffer;
```

```cpp
std_msgs::msg::Int32MultiArray
typedef struct GameFeedback_s
{
    int enemy_team_color;
    0;
    0

} GameFeedbackBuffer;
```

```cpp
    /**
    * @brief 发布比赛信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
    void PublishGameInfo(
        uint8_t* buffer,
        rclcpp::PublisherBase::SharedPtr pub1,
        rclcpp::PublisherBase::SharedPtr pub2
    );
```

### 红方血量

发布 裁判系统

```cpp
std_msgs::msg::Int32MultiArray
typedef struct RedRobotHPFeedback_s //红方机器人血量反馈接收数据
{
    int red_1_robot_HP; // 红 1 英雄机器人血量
    int red_2_robot_HP; // 红 2 工程机器人血量
    int red_3_robot_HP; // 红 3 步兵机器人血量
    int red_4_robot_HP; // 红 4 步兵机器人血量
    int red_5_robot_HP; // 红 5 步兵机器人血量
    int red_7_robot_HP; // 红 7 哨兵机器人血量
} RedRobotHPFeedbackBuffer;
```

```cpp
    /**
    * @brief 发布红方机器人血量信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
void PublishRedRobotHP(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);
```

### 蓝方血量

发布 裁判系统

```cpp
Int32MultiArray
typedef struct BlueRobotHPFeedback_s //蓝方机器人血量反馈接收数据
{
    int blue_1_robot_HP; // 蓝 1 英雄机器人血量
    int blue_2_robot_HP; // 蓝 2 工程机器人血量
    int blue_3_robot_HP; // 蓝 3 步兵机器人血量
    int blue_4_robot_HP; // 蓝 4 步兵机器人血量
    int blue_5_robot_HP; // 蓝 5 步兵机器人血量
    int blue_7_robot_HP; // 蓝 7 哨兵机器人血量
} BlueRobotHPFeedbackBuffer;
```

```cpp
    /**
    * @brief 发布蓝方机器人血量信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
void PublishBlueRobotHP(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);
```

### 建筑血量

发布 裁判系统

```cpp
Int32MultiArray
typedef struct BuildingHPFeedback_s //建筑血量反馈接收数据
{
    int red_outpost_HP;  // 红方前哨站血量
    int red_base_HP;     // 红方基地血量
    int blue_outpost_HP; // 蓝方前哨站血量
    int blue_base_HP;    // 蓝方基地血量
} BuildingHPFeedbackBuffer;
```

```cpp
    /**
    * @brief 发布建筑血量信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
void PublishBuildingHP(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);
```

### 发射状态量

发布 裁判系统

```cpp
std_msgs::msg::Int32MultiArray
typedef struct ShootStautsControl_s // 发射状态量发送数据
{
    int real_heat;     //发射机构热量
    int bullet_speed;  //弹速
    int game_progress; //比赛阶段
} ShootStautsBuffer;
```

```cpp
    /**
    * @brief 发布发射状态量信息话题
    * @param pub 发布者
    * @param buffer 发送数据
    */
void PublishShootStauts(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);
```

### 敌方机器人位置1

发布 裁判系统

```cpp
std_msgs::msg::Float32MultiArray
typedef struct EnemyRobotPositionFeedback_s // 敌方123位置反馈接收数据
{
    float enemy1_x_position; //敌方1号x轴坐标
    float enemy1_y_position; //敌方1号y轴坐标
    float enemy2_x_position; //敌方2号x轴坐标
    float enemy2_y_position; //敌方2号y轴坐标
    float enemy3_x_position; //敌方3号x轴坐标
    float enemy3_y_position; //敌方3号y轴坐标
} EnemyRobotPositionFeedbackBuffer;
```

```cpp
    /**
    * @brief 发布敌方机器人位置信息话题(1 2 3 号机器人)
    * @param pub 发布者
    * @param buffer 发送数据
    */
void PublishEnemyRobotPosition(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);
```

### 敌方机器人位置2

发布 裁判系统

```cpp
std_msgs::msg::Float32MultiArray
typedef struct EnemyExRobotPositionFeedback_s // 敌方457位置反馈接收数据
{
    float enemy4_x_position; //敌方4号x轴坐标
    float enemy4_y_position; //敌方4号y轴坐标
    float enemy5_x_position; //敌方5号x轴坐标
    float enemy5_y_position; //敌方5号y轴坐标
    float enemy7_x_position; //敌方7号x轴坐标
    float enemy7_y_position; //敌方7号y轴坐标
} EnemyExRobotPositionFeedbackBuffer;
```

```cpp
    /**
    * @brief 发布敌方机器人位置信息话题(4 5 7 号机器人)
    * @param pub 发布者
    * @param buffer 发送数据
    */
void PublishEnemyExRobotPosition(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);
```

## 原位置:烧饼downlink

回调 串口

### 云台控制回调

```cpp
typedef struct PtzControl_s // 云台控制发送数据
{
    char find_bool; // 追踪
    float yaw;      // 偏航角
    float pitch;    // 俯仰角
    float distance; // 距离
} PtzControlBuffer;
```

```cpp
    /**
     * @brief 订阅云台控制话题回调函数
     */
void PTZCB(const communicate::msg::SerialInfo::SharedPtr msg);
```

### 大云台控制回调

```cpp
typedef struct MainptzControl_s // 大云台控制发送数据
{
    float yaw; // 偏航角
} MainptzControlBuffer;
```

```cpp
    /**
     * @brief 订阅大云台控制话题回调函数
     */
void MainPTZCB(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
```

### 底盘速度控制回调

```cpp
typedef struct ChassisControl_s // 底盘控制发送数据
{
    float vx; // x 轴方向速度
    float vy; // y 轴方向速度
} ChassisControlBuffer;
```

```cpp
    /**
     * @brief 订阅底盘速度控制话题回调函数
     */
void ChassisVelCB(const geometry_msgs::msg::Twist::SharedPtr msg);
```

### 比赛交互控制回调

```cpp
typedef struct InteractionControl_s // 比赛交互控制发送数据
{
    int type;    // 类型  0：无  1：复活  2：买弹丸
    int content; // 具体内容
} InteractionControlBuffer;
```

```cpp
    /**
     * @brief 订阅比赛交互控制话题回调函数
     */
void InteractionCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
```

### 车体模块控制回调

```cpp
typedef struct MoudleControl_s // 车体模块控制发送数据
{
    int type;    // 类型  0：无  1：小陀螺  2：云台单连发控制
    int content; // 具体内容
} MoudleControlBuffer;
```

```cpp
    /**
     * @brief 订阅车体模块控制话题回调函数
     */
void MoudleCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
```

### 发射状态量回调

```cpp
typedef struct ShootStautsControl_s // 发射状态量发送数据
{
    int real_heat;     //发射机构热量
    int bullet_speed;  //左弹速
    int game_progress; //比赛阶段
} ShootStautsBuffer;
```

```cpp
    /**
     * @brief 订阅发射状态量话题回调函数
     */
void ShootStautsCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
```

## 原位置:自瞄uplink

### 自瞄假信息

发布 串口

std_msgs::msg::Int32MultiArray

```cpp
enemy_team_color;
mode;
rune_flag;
```

```cpp
    /**
     * @brief 发布自瞄假信息话题
     * @param pub 发布者
     */
void PublishAutoaimDefault(
    rclcpp::PublisherBase::SharedPtr pub
);
```

### 自瞄信息

包含云台控制和模式切换

发布 串口

```text
pub1 : std_msgs::msg::Int32MultiArray    // 未使用
pub2 : sensor_msgs::msg::JointState      // msg_r
pub3 : sensor_msgs::msg::JointState      // msg_l
pub4 : sensor_msgs::msg::JointState      // msg_gyro
```

```cpp
stamp : now()
header.frame_id : data->enemy_team_color
typedef struct AutoaimFeedback_s  // 自瞄信息反馈接收数据
{
    char enemy_team_color;  // 敌方颜色 'r':红色 'b':蓝色
    char mode;              // 模式 ‘a’：自瞄 'r'：符
    char rune_flag;         // 符模式 '0'：不可激活 '1'：小符 '2':大符
    float left_yaw;         // 偏航角 左云台
    float left_pitch;       // 俯仰角
    float right_yaw;        // 偏航角 右云台
    float right_pitch;      // 俯仰角
    float yaw;              // 偏航角 大云台
} AutoaimFeedbackBuffer;
```

```cpp
    /**
     * @brief 发布自瞄信息信息话题
     * @param pub 发布者
     * @param buffer 发送数据
     */
void PublishAutoaim(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub1,
    rclcpp::PublisherBase::SharedPtr pub2,
    rclcpp::PublisherBase::SharedPtr pub3,
    rclcpp::PublisherBase::SharedPtr pub4
);
```

result ： communicate::srv::ModeSwitch::Request

```cpp
ModeSwitch.srv
int8 mode        // 模式 0：自瞄 1：符 
int8 rune_state  // 符模式 0：不可激活 1：小符 2:大符
int8 enemy_color // 敌方颜色 0：红 1：蓝


---
bool success     // 返回是否成功
```

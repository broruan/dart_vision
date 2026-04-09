# 功能描述

## 注意事项

敌方颜色要写死

允许发弹量不区分 17mm 弹丸允许发弹量 和 42mm 弹丸允许发弹量

24.10.21 : 5号步兵被删除，但串口协议未更新，因此暂时保留

节点名称： communicate_node

## 调试(发上位机)

### tf树假信息

/communicate/debug/gyro

sensor_msgs::msg::JointState

GYRO_DEFAULT

```cpp
stamp = now();
position = (0, 0, )
```

### 自瞄假信息

/communicate/debug/autoaim

communicate_2025::msg::Autoaim

AUTOAIM_DEFAULT

```cpp
stamp = now()
float yaw=0;
float pitch=0;
enemy_team_color = 0;
mode = 0;
rune_flag = 0;
```

## 上位机发下位机

### 云台控制回调

/shoot_info

```cpp
typedef struct GimbalControl_s // 云台控制发送数据
{
    char find_bool; // 追踪
    float yaw;      // 偏航角
    float pitch;    // 俯仰角
} GimbalControl;
```

### 底盘速度控制回调

sentry/cmd_vel

```cpp
typedef struct ChassisControl_s // 底盘控制发送数据
{
    float x_speed; // x 轴方向速度
    float y_speed; // y 轴方向速度
    float yaw;     // 大 yaw
} ChassisControl;
```

### 比赛交互控制回调

/behaviortree/interaction

```cpp
typedef struct InteractionControl_s // 比赛交互控制发送数据
{
    int type;    // 类型  0：无  1：复活  2：买弹丸
    int content; // 具体内容
} InteractionControl;
```

### 车体模块控制回调

/behaviortree/moudle

```cpp
typedef struct MoudleControl_s // 车体模块控制发送数据
{
    int type;    // 类型  0：无  1：小陀螺  2：云台单连发控制
    int content; // 具体内容
} MoudleControl;
```

### 工程机械臂控制回调

/engineer/arm_conttrol

```cpp
typedef struct EngineerArm_s // 工程机械臂控制
{ 
    float h0; // 龙门架抬升
    float x1; // 龙门架滑轨

    float q2; // 肩关节
    float q3; // 肘关节
    float q4; // 腕关节旋转yaw
    float q5; // 腕关节旋转pitch
    float q6; // 腕关节旋转roll
} EngineerArmControl;
```

```cpp
    /**
     * @brief 订阅工程机械臂控制话题回调函数
     */
void EngineerArmCB(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
```

### 工程交互控制回调

/engineer/interaction_conttrol

```cpp
typedef struct EngineerInteraction_s // 工程交互控制
{ 
    int type;    // 类型  0:吸盘 1:兑换操作
    int content; // 具体内容
} EngineerInteractionControl;
```

## 下位机发上位机

### 自瞄云台信息

/communicate/autoaim

communicate_2025::msg::Autoaim

AUTOAIM

```cpp
header.stamp = this->now()
header.frame_id = "shooter"
typedef struct Autoaim_s  // 自瞄云台信息接收数据
{
    float yaw;                 // 偏航角
    float pitch;               // 俯仰角
    uint16_t enemy_team_color; // 敌方颜色 0 ：红 1 ：蓝
    uint16_t mode;             // 模式 0 ：自瞄 1 ：符 2 : 部署模式
    uint16_t rune_flag;        // 符模式 '0'：不可激活 '1'：小符 '2':大符
} Autoaim;

```

### 我方机器人位置1

/communicate/position/robot

std_msgs::msg::Float32MultiArray

ROBOT_POSITION

```cpp
typedef struct RobotPosition_s
{
    float infantry_3_x; // 己方 3 号步兵机器人位置 x 轴坐标
    float infantry_3_y; // 己方 3 号步兵机器人位置 y 轴坐标
    float infantry_4_x; // 己方 4 号步兵机器人位置 x 轴坐标
    float infantry_4_y; // 己方 4 号步兵机器人位置 y 轴坐标
    float infantry_5_x; // 己方 5 号步兵机器人位置 x 轴坐标
    float infantry_5_y; // 己方 5 号步兵机器人位置 y 轴坐标
} RobotPosition;
```

### 我方机器人位置2

/communicate/position/exrobot

std_msgs::msg::Float32MultiArray

ROBOT_EX_POSITION

```cpp
typedef struct ExRobotPosition_s
{
    float hero_x;     // 己方英雄机器人位置 x 轴坐标
    float hero_y;     // 己方英雄机器人位置 y 轴坐标
    float engineer_x; // 己方工程机器人位置 x 轴坐标
    float engineer_y; // 己方工程机器人位置 y 轴坐标
    float sentinel_x; // 己方哨兵机器人位置 x 轴坐标
    float sentinel_y; // 己方哨兵机器人位置 y 轴坐标
} ExRobotPosition;
```

### 敌方机器人位置1

/communicate/position/enemyrobot

std_msgs::msg::Float32MultiArray

ENEMY_ROBOT_POSITION

```cpp
typedef struct EnemyRobotPosition_s // 敌方123位置反馈接收数据
{
    float enemy_infantry_3_x; // 敌方 3 号步兵机器人位置 x 轴坐标
    float enemy_infantry_3_y; // 敌方 3 号步兵机器人位置 y 轴坐标
    float enemy_infantry_4_x; // 敌方 4 号步兵机器人位置 x 轴坐标
    float enemy_infantry_4_y; // 敌方 4 号步兵机器人位置 y 轴坐标
    float enemy_infantry_5_x; // 敌方 5 号步兵机器人位置 x 轴坐标
    float enemy_infantry_5_y; // 敌方 5 号步兵机器人位置 y 轴坐标
} EnemyRobotPosition;

```

### 敌方机器人位置2

/communicate/position/enemyexrobot

std_msgs::msg::Float32MultiArray

ENEMY_EX_ROBOT_POSITION

```cpp
typedef struct ExEnemyRobotPosition_s // 敌方67位置反馈接收数据
{
    float enemy_hero_x;     // 敌方英雄机器人位置 x 轴坐标
    float enemy_hero_y;     // 敌方英雄机器人位置 y 轴坐标
    float enemy_engineer_x; // 敌方工程机器人位置 x 轴坐标
    float enemy_engineer_y; // 敌方工程机器人位置 y 轴坐标
    float enemy_sentinel_x; // 敌方哨兵机器人位置 x 轴坐标
    float enemy_sentinel_y; // 敌方哨兵机器人位置 y 轴坐标
} ExEnemyRobotPosition;
```

### 红方血量

/communicate/hp/redrobot

std_msgs::msg::Int32MultiArray

RED_ROBOT_HP

```cpp
typedef struct RedRobotHP_s // 红方机器人血量反馈接收数据
{
    int red_1_robot_HP; // 红 1 英雄机器人血量
    int red_2_robot_HP; // 红 2 工程机器人血量
    int red_3_robot_HP; // 红 3 步兵机器人血量
    int red_4_robot_HP; // 红 4 步兵机器人血量
    int red_5_robot_HP; // 红 5 步兵机器人血量
    int red_7_robot_HP; // 红 7 哨兵机器人血量
} RedRobotHP;
```

### 蓝方血量

/communicate/hp/bluerobot

std_msgs::msg::Int32MultiArray

BLUE_ROBOT_HP

```cpp
typedef struct BlueRobotHP_s // 蓝方机器人血量反馈接收数据
{
    int blue_1_robot_HP; // 蓝 1 英雄机器人血量
    int blue_2_robot_HP; // 蓝 2 工程机器人血量
    int blue_3_robot_HP; // 蓝 3 步兵机器人血量
    int blue_4_robot_HP; // 蓝 4 步兵机器人血量
    int blue_5_robot_HP; // 蓝 5 步兵机器人血量
    int blue_7_robot_HP; // 蓝 7 哨兵机器人血量
} BlueRobotHP;
```

### 建筑血量

/communicate/hp/building

std_msgs::msg::Int32MultiArray

BUILDING_HP

```cpp
typedef struct BuildingHP_s // 建筑血量反馈接收数据
{
    int red_outpost_HP;  // 红方前哨站血量
    int red_base_HP;     // 红方基地血量
    int blue_outpost_HP; // 蓝方前哨站血量
    int blue_base_HP;    // 蓝方基地血量
} BuildingHP;

```

### 比赛信息

/communicate/gameinfo

std_msgs::msg::Int32MultiArray

GAME_INFO

```cpp
typedef struct GameInfo_s // 比赛反馈接收数据
{
    int enemy_team_color;    // 敌方颜色 0：红 1：蓝
    int game_progress;       // 当前比赛阶段
    int stage_remain_time;   // 当前阶段剩余时间
    int remaining_gold_coin; // 剩余金币数量
} GameInfo;
```

### 发射状态量

/communicate/shootstatus

std_msgs::msg::Int32MultiArray

SHOOT_STATUS

```cpp
typedef struct ShootStatus_s // 发射状态量发送数据 
{
    int projectile_allowance_17mm; // 17mm允许发弹量
    int projectile_allowance_42mm; // 42mm允许发弹量
    int real_heat;                 // 发射机构热量
    int launching_frequency;       // 弹丸射速(单位:Hz)
} ShootStatus;
```

### 操作反馈

/communicate/command

communicate_2025::msg::Command

COMMAND

```cpp
typedef struct Command_s //操作反馈发送数据
{
    float target_position_x; // 目标位置 x 轴坐标
    float target_position_y; // 目标位置 y 轴坐标
    int cmd_keyboard;        // 键盘信息
    int target_robot_id;     // 目标机器人id
}Command;
```

### 受击反馈

/communicate/hitted

std_msgs::msg::Int32MultiArray

HITTED

```cpp
typedef struct Hitted_s //受击反馈发送数据
{
    int hitted; // 受击打标识

}Hitted;
```

### 工程机械臂

/communicate/engineerarm

std_msgs::msg::Float32MultiArray

ENGINEER_ARM

```cpp
typedef struct EngineerArm_s // 工程机械臂发送数据
{
    float h0; // 龙门架抬升
    float x1; // 龙门架滑轨

    float q2; // 肩关节
    float q3; // 肘关节
    float q4; // 腕关节旋转yaw
    float q5; // 腕关节旋转pitch
    float q6; // 腕关节旋转roll
}EngineerArm;
```

### 工程交互

/communicate/engineerinteraction

std_msgs::msg::Int32MultiArray

ENGINEER_INTERACTION

```cpp
typedef struct EngineerInteraction_s // 工程交互发送数据
{
    int type;    // 类型  0：无  1：启动兑矿  2：结束兑矿
    int content; // 保留内容
}EngineerInteraction;
```

## 自定义消息类型

### Autoaim.msg

```text
std_msgs/Header header

float32 pitch
float32 high_gimbal_yaw
uint8 enemy_team_color
uint8 mode
uint8 rune_flag
float32 low_gimbal_yaw
```

### Command.msg

```text
float32 target_position_x
float32 target_position_y
uint8 cmd_keyboard
uint8 target_robot_id
```

### SerialInfo.msg

```text
float32 yaw
float32 pitch
std_msgs/Char is_find
```

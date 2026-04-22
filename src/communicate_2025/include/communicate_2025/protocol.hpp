#pragma once

#include <cstdint>
#include <vector>

#pragma pack(push, 1)
typedef struct Protocol {
    uint8_t start;      // 帧头，取 's’     0
    uint8_t type;       // 消息类型         1
                        /* type 取值：
                         *      步兵：
                         *       上位机发下位机
                         *      0xA0     自瞄云台控制
                         *       下位机发上位机
                         *      0xB0     自瞄云台反馈

                         *      哨兵：
                         *       上位机发下位机
                         *      0xA0     自瞄云台控制
                         *      0xA6     自瞄云台控制（含距离），飞镖用
                         *      0xA1     底盘控制
                         *      0xA2     比赛交互控制
                         *      0xA3     车体模块控制
                         *       下位机发上位机
                         *      0xB0     自瞄云台反馈
                         *      0xB1     友方机器人位置1反馈
                         *      0xB2     友方机器人位置2反馈
                         *      0xB3     敌方机器人位置1反馈
                         *      0xB4     敌方机器人位置2反馈
                         *      0xB5     红方机器人血量反馈
                         *      0xB6     蓝方机器人血量反馈
                         *      0xB7     建筑血量反馈
                         *      0xB8     比赛信息反馈
                         *      0xB9     操作信息反馈
                         *      0xBA     受击信息反馈
                         *      0xBB     发射状态量反馈

                         *      英雄：
                         *       上位机发下位机
                         *      0xA0     自瞄云台控制
                         *       下位机发上位机
                         *      0xB0     自瞄云台反馈
                         *      0xBB     发射状态量反馈

                         *      工程:
                         *       上位机发下位机
                         *      0xA4     机械臂控制
                         *      0xA5     交互控制
                         *       下位机发上位机
                         *      0xBC     机械臂反馈
                         *      0xBD     比赛信息反馈
                         */
    uint8_t buffer[29]; // 数据            2 - 30
    uint8_t end;        // 帧尾，取 'e'    31
} Message;
#pragma pack(pop)

inline Message fromVector(const std::vector<uint8_t>& data) {
    Message packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t*>(&packet));
    return packet;
}

inline std::vector<uint8_t> toVector(const Message& data) {
    std::vector<uint8_t> packet(sizeof(Message));
    std::copy(
        reinterpret_cast<const uint8_t*>(&data),
        reinterpret_cast<const uint8_t*>(&data) + sizeof(Message),
        packet.begin()
    );
    return packet;
}

#pragma pack(push, 1)

// 0xB0 自瞄云台反馈
typedef struct Autoaim_s {
    float high_gimbal_yaw;
    float pitch;
    uint8_t enemy_team_color;
    uint8_t mode;
    uint8_t rune_flag;
    float low_gimbal_yaw;
} Autoaim;

// 0xA0 自瞄云台控制
typedef struct GimbalControl_s {
    char find_bools;
    float yaw;
    float pitch;
} GimbalControl;

// 0xA6 自瞄云台控制（含初速度，飞镖用）
typedef struct GimbalControlWithVel_s {
    char find_bools;
    float yaw;
    float pitch;
    float s; // 飞镖所需拉力的距离
} GimbalControlWithVel;

// 0xA1 底盘控制
typedef struct ChassisControl_s {
    float x_speed;
    float y_speed;
    float yaw;
} ChassisControl;

// 0xA2 比赛交互控制
typedef struct InteractionControl_s {
    uint8_t type;    // 类型  0：无  1：买活  2：买弹丸
    uint8_t content; // 具体内容
} InteractionControl;

// 0xA3 车体模块控制
typedef struct MoudleControl_s {
    uint8_t type;    // 类型  0：无  1：小陀螺  2：单连发控制
    uint8_t content; // 具体内容
} MoudleControl;

// 0xA4 工程机械臂控制
typedef struct EngineerArmControl_s {
    double h0; // 龙门架抬升
    double x1; // 龙门架滑轨

    double q2; // 肩关节
    double q3; // 肘关节
    double q4; // 腕关节旋转yaw
    double q5; // 腕关节旋转pitch
    double q6; // 腕关节旋转roll
} EngineerArmControl;

// 0xA5 工程交互控制
typedef struct EngineerInteractionControl_s {
    uint8_t type;    // 类型  0:吸盘 1:兑换操作
    uint8_t content; // 具体内容
} EngineerInteractionControl;

// 0xB1 己方步兵位置反馈
typedef struct RobotPosition_s {
    float infantry_3_x; // 己方 3 号步兵机器人位置 x 轴坐标
    float infantry_3_y; // 己方 3 号步兵机器人位置 y 轴坐标
    float infantry_4_x; // 己方 4 号步兵机器人位置 x 轴坐标
    float infantry_4_y; // 己方 4 号步兵机器人位置 y 轴坐标
    float infantry_5_x; // 己方 5 号步兵机器人位置 x 轴坐标
    float infantry_5_y; // 己方 5 号步兵机器人位置 y 轴坐标
} RobotPosition;

// 0xB2 己方英雄，工程，哨兵（按顺序）位置反馈
typedef struct ExRobotPosition_s {
    float hero_x;     // 己方英雄机器人位置 x 轴坐标
    float hero_y;     // 己方英雄机器人位置 y 轴坐标
    float engineer_x; // 己方工程机器人位置 x 轴坐标
    float engineer_y; // 己方工程机器人位置 y 轴坐标
    float sentinel_x; // 己方哨兵机器人位置 x 轴坐标
    float sentinel_y; // 己方哨兵机器人位置 y 轴坐标
} ExRobotPosition;

// 0xB3 敌方步兵位置反馈
typedef struct EnemyRobotPosition_s {
    float enemy_infantry_3_x; // 敌方 3 号步兵机器人位置 x 轴坐标
    float enemy_infantry_3_y; // 敌方 3 号步兵机器人位置 y 轴坐标
    float enemy_infantry_4_x; // 敌方 4 号步兵机器人位置 x 轴坐标
    float enemy_infantry_4_y; // 敌方 4 号步兵机器人位置 y 轴坐标
    float enemy_infantry_5_x; // 敌方 5 号步兵机器人位置 x 轴坐标
    float enemy_infantry_5_y; // 敌方 5 号步兵机器人位置 y 轴坐标
} EnemyRobotPosition;

// 0xB4 敌方英雄，工程，哨兵（按顺序）位置反馈
typedef struct ExEnemyRobotPosition_s {
    float enemy_hero_x;     // 敌方英雄机器人位置 x 轴坐标
    float enemy_hero_y;     // 敌方英雄机器人位置 y 轴坐标
    float enemy_engineer_x; // 敌方工程机器人位置 x 轴坐标
    float enemy_engineer_y; // 敌方工程机器人位置 y 轴坐标
    float enemy_sentinel_x; // 敌方哨兵机器人位置 x 轴坐标
    float enemy_sentinel_y; // 敌方哨兵机器人位置 y 轴坐标
} ExEnemyRobotPosition;

// 0xB5 红方机器人血量反馈
typedef struct RedRobotHP_s {
    uint16_t red_1_robot_HP; // 红 1 英雄机器人血量
    uint16_t red_2_robot_HP; // 红 2 工程机器人血量
    uint16_t red_3_robot_HP; // 红 3 步兵机器人血量
    uint16_t red_4_robot_HP; // 红 4 步兵机器人血量
    uint16_t red_5_robot_HP; // 红 5 步兵机器人血量
    uint16_t red_7_robot_HP; // 红 7 哨兵机器人血量
} RedRobotHP;

// 0xB6 蓝方机器人血量反馈
typedef struct BlueRobotHP_s {
    uint16_t blue_1_robot_HP; // 蓝 1 英雄机器人血量
    uint16_t blue_2_robot_HP; // 蓝 2 工程机器人血量
    uint16_t blue_3_robot_HP; // 蓝 3 步兵机器人血量
    uint16_t blue_4_robot_HP; // 蓝 4 步兵机器人血量
    uint16_t blue_5_robot_HP; // 蓝 5 步兵机器人血量
    uint16_t blue_7_robot_HP; // 蓝 7 哨兵机器人血量
} BlueRobotHP;

// 0xB7 建筑血量反馈
typedef struct BuildingHP_s {
    uint16_t red_outpost_HP;  // 红方前哨站血量
    uint16_t red_base_HP;     // 红方基地血量
    uint16_t blue_outpost_HP; // 蓝方前哨站血量
    uint16_t blue_base_HP;    // 蓝方基地血量
} BuildingHP;

// 0xB8 比赛信息反馈
typedef struct GameInfo_s {
    uint8_t enemy_team_color;     // 敌方颜色 0：红 1：蓝
    uint8_t game_progress;        // 当前比赛阶段    0x0001
    uint16_t stage_remain_time;   // 当前阶段剩余时间 0x0001
    uint16_t remaining_gold_coin; // 剩余金币数量    0x0208
} GameInfo;

// 0xB9 操作信息反馈
typedef struct Command_s {
    float target_position_x; // 目标位置 x 轴坐标
    float target_position_y; // 目标位置 y 轴坐标
    uint8_t cmd_keyboard;    // 键盘信息
    uint8_t target_robot_id; // 目标机器人id
} Command;

// 0xBA 受击信息反馈
typedef struct Hitted_s {
    uint8_t hitted; // 受击打标识

} Hitted;

// 0xBB 发射状态量反馈
typedef struct ShootStatus_s {
    uint16_t projectile_allowance_17mm; // 17mm允许发弹量
    uint16_t projectile_allowance_42mm; // 42mm允许发弹量
    uint8_t real_heat;                  // 发射机构热量
    uint8_t launching_frequency;        // 弹丸射频(单位：Hz)
} ShootStatus;

typedef struct EngineerArm_s // 工程机械臂发送数据
{
    float h0; // 龙门架抬升
    float x1; // 龙门架滑轨

    float q2; // 肩关节
    float q3; // 肘关节
    float q4; // 腕关节旋转yaw
    float q5; // 腕关节旋转pitch
    float q6; // 腕关节旋转roll
} EngineerArm;

typedef struct EngineerInteraction_s // 工程交互发送数据
{
    uint16_t type;    // 类型  0：无  1：启动兑矿  2：结束兑矿
    uint16_t content; // 保留内容
} EngineerInteraction;

#pragma pack(pop)

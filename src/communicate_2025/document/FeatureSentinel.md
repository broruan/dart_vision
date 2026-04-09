# FeatureSentinel

## 请根据所需要的消息选择合适的type

## 上位机发下位机

### 0xA0 自瞄控制

```cpp
    char start;     // 0 帧头，取 's'
    char datatype;  // 1 消息类型 0xA0
    char find_bool; // 2 是否追踪
    float yaw;      // 3 - 6 偏航角
    float pitch;    // 7 - 10 俯仰角
    ...             // 11 - 30 预留空位
    char end;       // 31 帧尾，取 'e'
```

### 0xA1 底盘控制

```cpp
    char start;     // 0 帧头，取 's'
    char datatype;  // 1 消息类型 0xA1
    float x_speed;  // 2 - 5 x方向速度
    float y_speed;  // 6 - 9 y方向速度
    float yaw       // 10 - 13 yaw
    ...             // 14 - 30 预留空位
    char end;       // 31 帧尾，取 'e'
```

### 0xA2 比赛交互控制

```cpp
    char start;       // 0 帧头，取 's'
    char datatype;    // 1 消息类型 0xA2
    uint8_t type;     // 2 类型  0：无  1：复活  2：买弹丸
    uint8_t content;  // 3 具体内容
    ...               // 4 - 30 预留空位
    char end;         // 31 帧尾，取 'e'
```

### 0xA3 车体模块控制

```cpp
    char start;       // 0 帧头，取 's'
    char datatype;    // 1 消息类型 0xA3
    uint16_t type;    // 2 类型  0：无  1：小陀螺  2：云台单连发控制
    uint16_t content; // 3 具体内容
    ...               // 4 - 30 预留空位
    char end;         // 31 帧尾，取 'e'
```

## 下位机发上位机

### 0xB0 自瞄云台反馈

```cpp
    char start;               // 0 帧头，取 's'
    char datatype;            // 1 消息类型 0xB0
    float high_gimbal_yaw;    // 2 - 5 小云台偏航角
    float pitch;              // 6 - 9 俯仰角
    uint8_t enemy_team_color; // 10 敌方颜色 0 ：红 1 ：蓝
    uint8_t mode;             // 11 模式 0 ：自瞄 1 ：符
    uint8_t rune_flag;        // 12 符模式 '0'：不可激活 '1'：小符 '2':大符
    float low_gimbal_yaw;     // 13 - 16 大云台偏航角
    ...                       // 17 - 30 预留空位
    char end;                 // 31 帧尾，取 'e'
```

### 0xB1 我方机器人位置信息话题1

```cpp
    char start;         // 0 帧头，取 's'
    char datatype;      // 1 消息类型 0xB1
    float infantry_3_x; // 2 - 5 己方 3 号步兵机器人位置 x 轴坐标
    float infantry_3_y; // 6 - 9 己方 3 号步兵机器人位置 y 轴坐标
    float infantry_4_x; // 10 - 13 己方 4 号步兵机器人位置 x 轴坐标
    float infantry_4_y; // 14 - 17 己方 4 号步兵机器人位置 y 轴坐标
    float infantry_5_x; // 18 - 21 己方 5 号步兵机器人位置 x 轴坐标
    float infantry_5_y; // 22 - 25 己方 5 号步兵机器人位置 y 轴坐标
    ...                 // 26 - 30 预留空位
    char end;           // 31 帧尾，取 'e'
```

### 0xB2 我方机器人位置信息话题2

```cpp
    char start;       // 0 帧头，取 's'
    char datatype;    // 1 消息类型 0xB2
    float hero_x;     // 2 - 5 己方 1 号英雄机器人位置 x 轴坐标
    float hero_y;     // 6 - 9 己方 1 号英雄机器人位置 y 轴坐标
    float engineer_x; // 10 - 13 己方 2 号工程机器人位置 x 轴坐标
    float engineer_y; // 14 - 17 己方 2 号工程机器人位置 y 轴坐标
    float sentinal_x; // 18 - 21 己方 7 号哨兵机器人位置 x 轴坐标
    float sentinal_y; // 22 - 25 己方 7 号哨兵机器人位置 y 轴坐标
    ...               // 26 - 30 预留空位
    char end;         // 31 帧尾，取 'e'
```

### 0xB3 敌方机器人位置1

```cpp
    char start;               // 0 帧头，取 's'
    char datatype;            // 1 消息类型 0xB3
    float enemy_infantry_3_x; // 2 - 5 敌方 3 号步兵机器人位置 x 轴坐标
    float enemy_infantry_3_y; // 6 - 9 敌方 3 号步兵机器人位置 y 轴坐标
    float enemy_infantry_4_x; // 10 - 13 敌方 4 号步兵机器人位置 x 轴坐标
    float enemy_infantry_4_y; // 14 - 17 敌方 4 号步兵机器人位置 y 轴坐标
    float enemy_infantry_5_x; // 18 - 21 敌方 5 号步兵机器人位置 x 轴坐标
    float enemy_infantry_5_y; // 22 - 25 敌方 5 号步兵机器人位置 y 轴坐标
    ...                       // 26 - 30 预留空位
    char end;                 // 31 帧尾，取 'e'
```

### 0xB4 敌方机器人位置2

```cpp
    char start;               // 0 帧头，取 's'
    char datatype;            // 1 消息类型 0xB4
    float enemy_infantry_1_x; // 2 - 5 敌方 1 号英雄机器人位置 x 轴坐标
    float enemy_infantry_1_y; // 6 - 9 敌方 1 号英雄机器人位置 y 轴坐标
    float enemy_infantry_2_x; // 10 - 13 敌方 2 号工程机器人位置 x 轴坐标
    float enemy_infantry_2_y; // 14 - 17 敌方 2 号工程机器人位置 y 轴坐标
    float enemy_infantry_7_x; // 18 - 21 敌方 7 号哨兵机器人位置 x 轴坐标
    float enemy_infantry_7_y; // 22 - 25 敌方 7 号哨兵机器人位置 y 轴坐标
    ...                       // 26 - 30 预留空位
    char end;                 // 31 帧尾，取 'e'
```

### 0xB5 红方血量

```cpp
    char start;              // 0 帧头，取 's'
    char datatype;           // 1 消息类型 0xB5
    uint16_t red_1_robot_HP; // 2 - 3 红 1 英雄机器人血量
    uint16_t red_2_robot_HP; // 4 - 5 红 2 工程机器人血量
    uint16_t red_3_robot_HP; // 6 - 7 红 3 步兵机器人血量
    uint16_t red_4_robot_HP; // 8 - 9 红 4 步兵机器人血量
    uint16_t red_5_robot_HP; // 10 - 11 红 5 步兵机器人血量
    uint16_t red_7_robot_HP; // 12 - 13 红 7 哨兵机器人血量
    ...                      // 14 - 30 预留空位
    char end;                // 31 帧尾，取 'e'
```

### 0xB6 蓝方血量

```cpp
    char start;               // 0 帧头，取 's'
    char datatype;            // 1 消息类型 0xB6
    uint16_t blue_1_robot_HP; // 2 - 3 蓝 1 英雄机器人血量
    uint16_t blue_2_robot_HP; // 4 - 5 蓝 2 工程机器人血量
    uint16_t blue_3_robot_HP; // 6 - 7 蓝 3 步兵机器人血量
    uint16_t blue_4_robot_HP; // 8 - 9 蓝 4 步兵机器人血量
    uint16_t blue_5_robot_HP; // 10 - 11 蓝 5 步兵机器人血量
    uint16_t blue_7_robot_HP; // 12 - 13 蓝 7 哨兵机器人血量
    ...                       // 14 - 30 预留空位
    char end;                 // 31 帧尾，取 'e'

```

### 0xB7 建筑血量

```cpp
    char start;               // 0 帧头，取 's'
    char datatype;            // 1 消息类型 0xB7
    uint16_t red_outpost_HP;  // 2 - 3  红方前哨站血量
    uint16_t red_base_HP;     // 4 - 5  红方基地血量
    uint16_t blue_outpost_HP; // 6 - 7 蓝方前哨站血量
    uint16_t blue_base_HP;    // 8 - 9 蓝方基地血量
    ...                       // 10 - 30 预留空位
    char end;                 // 31 帧尾，取 'e'
```

### 0xB8 比赛信息

```cpp
    char start;                   // 0 帧头，取 's'
    char datatype;                // 1 消息类型 0xB8
    uint8_t enemy_team_color;     // 2 敌方颜色 0：红 1：蓝
    uint8_t game_progress;        // 3 当前比赛阶段
    uint16_t stage_remain_time;   // 4 - 5 当前阶段剩余时间
    uint16_t remaining_gold_coin; // 6 - 7 剩余金币数量
    ...                           // 8 - 30 预留空位
    char end;                     // 31 帧尾，取 'e'
```

### 0xB9 操作反馈

```cpp
    char start;              // 0 帧头，取 's'
    char datatype;           // 1 消息类型 0xB9
    float target_position_x; // 2 - 5 目标位置 x 轴坐标
    float target_position_y; // 6 - 9 目标位置 y 轴坐标
    uint8_t cmd_keyboard;    // 10 键盘信息
    uint8_t target_robot_id; // 11 目标机器人id
    ...                      // 12 - 30 预留空位
    char end                 // 31 帧尾，取 'e'
```

### 0xBA 受击反馈

```cpp
    char start;     // 0 帧头，取 's'
    char datatype;  // 1 消息类型 0xBA
    uint8_t hitted; // 2 受击打标识
    ...             // 3 - 30 预留空位
    char end        // 31 帧尾，取 'e'
```

### 0xBB 发射状态量

```cpp
    char start;                         // 0 帧头，取 's'
    char datatype;                      // 1 消息类型 0xBB
    uint16_t projectile_allowance_17mm; // 2 - 3 17mm允许发弹量
    uint16_t projectile_allowance_42mm; // 4 - 5 42mm允许发弹量
    uint8_t real_heat;                  // 6 发射机构热量 
    uint8_t launching_frequency;        // 7 弹丸射速(单位:Hz)
    ...                                 // 8 - 30 预留空位
    char end;                           // 31 帧尾，取 'e'
```

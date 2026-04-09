# FeatureInfantary

## 请根据所需要的消息选择合适的type

## 上位机发下位机

### 0xA0 自瞄云台控制

```cpp
    char start;      // 0 帧头，取 's'
    char type;       // 1 消息类型 0xA0
    char find_bool;  // 2 是否追踪
    float yaw;       // 3 - 6 偏航角
    float pitch;     // 7 - 10 俯仰角
    ...              // 11 - 30 预留空位
    char end;        // 31 帧尾，取 'e'
```

## 下位机发上位机

### 0xB0 自瞄数据反馈

```cpp
    char start;                // 0 帧头，取 's'
    char type;                 // 1 消息类型 0xB0
    float yaw;                 // 2 - 5 偏航角
    float pitch;               // 6 - 9 俯仰角
    uint8_t enemy_team_color; // 10 - 11 敌方颜色 0 ：红 1 ：蓝
    uint8_t mode;             // 12 - 13 模式 0 ：自瞄 1 ：符
    uint8_t rune_flag;        // 14 - 15 符模式 '0'：不可激活 '1'：小符 '2':大符
    ...                        // 16 - 30 预留空位
    char end;                  // 31 帧尾，取 'e'
```

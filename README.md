# Dart_Vision_26

基于 ROS2 的飞镖视觉识别与弹道解算系统.

## 项目结构

```
dart_vision/
├── src/
│   ├── mindvision_camera/    # 迈德威视相机驱动
│   ├── detector/            # 目标检测、跟踪与弹道解算
│   └── communicate_2025/     # 裁判系统通信与串口通信
```

## 功能模块

### 1. mindvision_camera
迈德威视工业相机驱动包，提供图像采集功能。

主要功能：
- 工业相机图像采集
- 相机标定支持
- 多相机配置管理

### 2. detector
目标检测与弹道解算核心模块。

主要功能：
- **位姿解算**：基于PnP算法计算目标的空间位置和姿态
- **EKF 跟踪**：扩展卡尔曼滤波器实现目标状态跟踪(>-<开发中，可能用不上)
- **弹道解算**：根据检测到的目标位置和距离，计算飞行轨迹和发射角度
- **与下位机通信**：将解算结果（yaw、pitch、距离）发送给下位机

核心节点：
- `video_detector_node`：图像处理与目标检测
- `EKF_node`：飞镖状态滤波跟踪
- `orbit_calculate_node`：弹道解算

话题通信：
| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/image_pub` | sensor_msgs::msg::Image | 相机原始图像 |
| `/detect_info` | communicate_2025::msg::SerialInfo | 检测与解算结果 |
| `/serial_info` | communicate_2025::msg::SerialInfo | 发送给下位机的数据 |
| `/deal_img` | detector::msg::DealImg | 处理图像结果 |
| `/velocity`| detector::msg::DealImg

### 3. communicate_2025
裁判系统通信与串口通信模块。

主要功能：
- 裁判系统数据解析
- 与下位机串口通信
- 指令收发

## 4. 编译启动

```bash
# 进入工作空间
cd ~/dart_vision

# 编译所有包
colcon build
```

## 运行

### 启动launch
```bash
ros2 launch detector launch.py
```

## 配置

检测器主要参数在 `src/detector/config/config.yaml` 中配置：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `LIGHT_RADIUS` | 目标半径 (m) | 0.03 |
| `mass` | 弹丸质量 (kg) | 0.2 |
| `area` | 迎风面积 (m²) | 0.0001 |
| `cd` | 风阻系数 | 0.3 |
| `rho` | 空气密度 (kg/m³) | 1.225 |
| `g` | 重力加速度 (m/s²) | 9.80665 |
| `max_possible_v0` | 最大初速度 (m/s) | 200.0 |

## 消息类型

### DealImg.msg
```yaml
float64 yaw        # 目标 yaw 角
float64 distance   # 目标距离
float64 pitch      # 目标 pitch 角
float64 s          # 弹道解算参数
float64 velocity   # 计算速度
bool found         # 是否检测到目标
```

### SerialInfo.msg
```yaml
std_msgs/Header header
float32 pitch           # 云台 pitch 角
float32 high_gimbal_yaw  # 云台 yaw 角
uint8 enemy_team_color  # 敌方队伍颜色
uint8 mode              # 模式
uint8 rune_flag         #符文标志
float32 low_gimbal_yaw   # 下云台 yaw 角
```

## 依赖

- ROS2 Jazzy
- OpenCV
- cv_bridge
- 迈德威视相机 SDK
- serial_driver
- asio

## 注意事项

1. 相机内参已标定存储于 `detector/include/detector/detect.hpp`
2. 敌方队伍颜色和兵种类型需在配置文件中手动设置
3. 调试模式下可查看 `/detect_info` 话题验证检测效果

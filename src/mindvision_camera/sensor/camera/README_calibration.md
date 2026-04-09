# 相机标定图像采集工具

这个工具用于通过相机节点采集标定板图像，用于相机标定。

## 功能特性

- 实时显示相机图像
- 按空格键拍照保存
- 自动命名和编号图像文件
- 显示已拍摄图像数量
- 按ESC键退出

## 使用方法

### 1. 编译项目

```bash
cd /home/chen/ExtendDisk/RM/2025_autoaim_latest
colcon build --packages-select camera
source install/setup.bash
```

### 2. 启动标定采集

```bash
# 方法1：使用launch文件启动（推荐）
ros2 launch camera camera_calibration.launch.py save_directory:=./my_calibration_images

# 方法2：分别启动节点
# 终端1：启动相机节点
ros2 run camera camera_for_calibrate

# 终端2：启动标定采集节点
ros2 run camera camera_calibration_capture --ros-args -p save_directory:=./calibration_images
```

### 3. 操作说明

1. 程序启动后会显示相机实时图像
2. 将标定板放在相机前，确保标定板清晰可见
3. 按**空格键**拍照保存当前图像
4. 移动标定板到不同位置和角度，继续拍照
5. 建议拍摄20-30张不同角度和位置的标定板图像
6. 按**ESC键**退出程序

### 4. 标定建议

为了获得好的标定结果，建议拍摄以下类型的图像：

- 标定板在图像中心
- 标定板在图像四个角落
- 标定板倾斜不同角度
- 标定板距离相机不同距离
- 确保标定板完全在图像内且清晰

## 参数配置

- `save_directory`: 图像保存目录（默认：`./calibration_images`）

## 输出文件

- 图像文件格式：`calibration_image_XXXX.jpg`
- 文件按顺序编号：0000, 0001, 0002...
- 保存在指定目录中

## 故障排除

1. **无法看到图像窗口**
   - 确保相机节点正常运行
   - 检查话题 `/image_pub` 是否有数据发布

2. **无法保存图像**
   - 检查保存目录是否有写入权限
   - 确保磁盘空间充足

3. **图像质量差**
   - 调整相机曝光时间
   - 改善光照条件
   - 确保标定板平整无损坏

## 后续标定步骤

采集完图像后，可以使用OpenCV的标定工具或ROS的camera_calibration包进行标定：

```bash
# 使用ROS标定工具
ros2 run camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/image_pub camera:=/camera
```

注意：请根据你的标定板实际规格调整 `--size` 和 `--square` 参数。

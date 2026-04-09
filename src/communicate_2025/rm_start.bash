# 定义常量
USB_RESET_CMD="lsusb | sed 's/:/ /g' | awk '{print \$2 \$4}' | sed 's/ /\//g' | xargs -I {} sudo usbreset /dev/bus/usb/{}"
TTYACM_PATH="/dev/ttyACM0"
SETUP_BASH="install/setup.bash"

# 重置 USB 设备
function reset_usb_devices() {
    eval "$USB_RESET_CMD"
}

# 设置权限
function set_permissions() {
    if [ -e "$TTYACM_PATH" ]; then
        sudo chmod 777 "$TTYACM_PATH"
    fi
}

# 执行安装脚本
function run_setup() {
    if [ -f "$SETUP_BASH" ]; then
        . "$SETUP_BASH"
    fi
}

# 启动 ROS2
function start_ros2() {
    ros2 launch communicate_2025 launch.py
}

# 主循环
while true; do
    # 重置 USB 设备
    reset_usb_devices

    sleep 1
    
    # 设置权限
    set_permissions

    # 执行安装脚本
    run_setup

    # 启动 ROS2
    start_ros2
done
#!/bin/bash

# 安装 ROS 2
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y ros-humble-desktop

# 安装和更新 rosdep
sudo apt install python3-rosdep -y
sudo rosdep init || true
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# 安装其他 ros 依赖
sudo apt-get install -y ros-humble-ament-cmake python3-colcon-common-extensions
sudo apt install -y ros-humble-serial-driver
sudo apt install -y ros-humble-asio-cmake-module
sudo apt install -y ros-humble-ament-cmake-auto
sudo apt install -y ros-humble-rosidl-default-generators

# 设置环境变量
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 构建工程
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
# 2025通讯模块

[![ROS2-humble](https://github.com/HDU-PHOENIX/communicate_2025/actions/workflows/main.yml/badge.svg)](https://github.com/HDU-PHOENIX/communicate_2025/actions/workflows/main.yml)


具体功能详见[FEATURES.md](document/FEATURES.md#)

## 文件结构

```text
.
├── config
|   └── config.yaml
|
├── document
|   ├── communicate.drawio
|   ├── FeatureInfantary.md
|   ├── FEATURES.md
|   ├── FeatureSentinel.md
|   └── OLD_FEATURES.md
|
├── include/communicate
|   ├── link.hpp
|   └── protocol.hpp
|
├── luanch
|   └── launch.py
|
├── msg
|   ├── Autoaim.msg
|   └── SerialInfo.msg
|
└── src  
    ├── communicate_node.cpp
    └── link.cpp
```

## 文件框架

### config

- config.yaml: 配置文件

### document

- communicate.drawio: 功能一览
- [FeatureInfantary.md](document/FeatureInfantary.md#featureinfantary): 供电控查看的文档
- [FeatureSentinel.md](document/FeatureSentinel.md#featureSentinel): 供电控查看的文档
- [FEATURES.md](document/FEATURES.md#): 功能描述文档
- [OLD_FEATURES.md](document/OLD_FEATURES.md#): 旧版本功能描述文档

### include/communicate

- protocol.hpp: 通信协议
- link.hpp: 串口和裁判系统连接

### launch

- launch.py: 启动脚本

### msg

- Autoaim.msg: 自瞄话题信息
- SerialInfo.msg: 自瞄串口信息

### src

- link.cpp: 串口和裁判系统连接实现
- communicate_node.cpp: 通信节点启动文件

### 其他

- CMakeLists.txt: 编译文件
- package.xml: 配置文件
- rm_start.bash: 启动脚本

## 依赖安装

### 实验性安装脚本

```bash
sudo bash ./setup.bash
```

1.ROS-humble:

推荐使用鱼香ROS安装

一个可能额外需要的包

```bash
sudo apt install ros-humble-rosidl-default-generators
```

2.asio

```bash
sudo apt install ros-humble-asio-cmake-module
sudo apt install ros-humble-ament-cmake-auto

```

3.serial_driver

```bash
sudo apt install ros-humble-serial-driver
```

## exit 一览

### link exit 一览

1. 串口重开失败 exit(-1)
2. 消息发送失败 exit(-2)
3. 消息接受失败 exit(-3)
4. 数据包校验失败 exit(-4)
5. IO口创建失败 exit(-5)

### 其他 exit 一览

1.没有串口 exit（-255） 建议开启调试模式

## 部署注意事项

- config debug模式默认关闭
- config 敌方队伍颜色0/1写死，需手动修改。
- config 记得写死兵种类型

## 需要根据规则书进行修改

### 10.22.15.10

5号步兵永远的离开了我们，联盟赛固定阵容137
R.I.P 永远怀念
但是串口没更新所以先不改

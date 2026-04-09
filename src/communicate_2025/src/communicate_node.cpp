#include <cstdint>
#include <unistd.h>

#include "communicate_2025/link.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // TODO: 和电控协商, 是否需要调整波特率. 根据本人测试, 对 c 板通信可用更高波特率
    uint32_t baud_rate = 115200; // 波特率

    // 配置流控制,不使用流控制
    drivers::serial_driver::FlowControl fc = drivers::serial_driver::FlowControl ::NONE;

    // 配置奇偶校验,不使用奇偶校验
    drivers::serial_driver::Parity pt = drivers::serial_driver::Parity::NONE;

    // 配置停止位,使用一个停止位
    drivers::serial_driver::StopBits sb = drivers::serial_driver::StopBits::ONE;

    // 创建串口配置对象,接受上面的配置参数
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

    // 创建通信节点
    auto node = std::make_shared<RMLink>(std::move(device_config));

    std::thread Link_thread([&node] {
        while (rclcpp::ok()) {
            node->Recv();
        }
    });
    Link_thread.detach();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
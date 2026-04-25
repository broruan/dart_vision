#include <rclcpp/rclcpp.hpp>
#include "detector/orbit_calculate.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Orbit::OrbitCalculate>();// 节点名称"orbti_calculate"
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
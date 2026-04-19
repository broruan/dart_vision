#include <tuple>
#include <rclcpp/rclcpp.hpp>
#include <detector/msg/deal_img.hpp>

namespace Orbit {
    class OrbitCalculate : public rclcpp::Node{
    public:
        OrbitCalculate();
        ~OrbitCalculate();
        /**
        * @brief 模拟飞行，返回到达目标水平距离时的高度
        */
        double simulateFlight(const double& pitch, const double& dist, const double& test_v0);
        /**
        * @brief 使用二分法求解所需的初速度,回调函数
        */
        void solveVelocity(const detector::msg::DealImg::SharedPtr msg);
        rclcpp::Publisher<detector::msg::DealImg>::SharedPtr vel_pub_;
        rclcpp::Subscription<detector::msg::DealImg>::SharedPtr result_sub;
    
    private:
        double g;                     // 重力加速度 (m/s^2)
        double dt;                    // 积分时间步长
        double K;                     // 阻力系数
        double tolerance;             // 误差容忍度 (m)
        double max_possible_v0;       // 发射机构能达到的最大初速度 (m/s)
    };

}
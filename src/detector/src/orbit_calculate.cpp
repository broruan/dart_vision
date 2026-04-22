#include <tuple>
#include "detector/orbit_calculate.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>


namespace Orbit {
    OrbitCalculate::OrbitCalculate():Node("orbti_calculate"){
        // 创建订阅
        result_sub = this->create_subscription<detector::msg::DealImg>("/deal_img", 
            rclcpp::SensorDataQoS().keep_last(1), 
            std::bind(&OrbitCalculate::solveVelocity, this, std::placeholders::_1));
        // 创建发布
        vel_pub_ = this->create_publisher<detector::msg::DealImg>("/velocity",10);
        this->g = this->declare_parameter("g",9.80665);
        this->dt = this->declare_parameter("dt",0.001);
        this->tolerance = this->declare_parameter("tolerance",0.005);
        this->max_possible_v0 = this->declare_parameter("max_possible_v0",150.0);
        this->mass = this->declare_parameter("mass", 0.2);
        this->n = this->declare_parameter("n", 0.5);
        this->k_s = this->declare_parameter("k_s", 0.80);
        this->K = this->declare_parameter("K", 0.000091875);
    };
    OrbitCalculate::~OrbitCalculate(){};
    double OrbitCalculate::simulateFlight(const double& pitch, const double& dist, const double& test_v0){
        double target_x = dist * std::cos(pitch);// 目标水平距离
        double x = 0.0;
        double y = 0.0;
        double vx = test_v0 * std::cos(pitch);
        double vy = test_v0 * std::sin(pitch);

        while (x < target_x) {
            double v = std::sqrt(vx * vx + vy * vy);
            double ax = -(this->K) * v * vx;
            double ay = -(this->g) - K * v * vy;

            vx += ax * this->dt;
            vy += ay * this->dt;
            x  += vx * this->dt;
            y  += vy * this->dt;

            // // 如果已经落地很深，提前终止(实际看场上的pitch)
            // if (y < -5.0) break; 
        }
        return y;
    };

    void OrbitCalculate::solveVelocity(const detector::msg::DealImg::SharedPtr msg){
        detector::msg::DealImg m;

        double target_y = msg->distance * std::sin(msg->pitch);// 发射口距离目标高度
        
        // 二分法搜索所需速度
        double min_v = 0.0;
        double max_v = this->max_possible_v0;
        double mid_v = 0.0;
        int max_iter = 1000;
        int iter = 0;
        double s = 0;

        while (iter < max_iter) {
            mid_v = (min_v + max_v) / 2.0;
            double sim_y = this->simulateFlight(msg->pitch, msg->distance, mid_v);
            // 检查误差
            if (std::abs(sim_y - target_y) < this->tolerance) {
                m.velocity = mid_v;
                RCLCPP_INFO(this->get_logger(), "velocity: %.2f  sim_y: %.2f  target_y: %.2f", mid_v, sim_y, target_y);
                break; 
            }
            // 速度与高度正相关：打低了就加速，打高了就减速
            if (sim_y < target_y) {
                min_v = mid_v; // 速度不够，打低了
            } else {
                max_v = mid_v; // 速度过大，打高了
            }
            iter++;
        }
        /** 
         *   s = v * std::aqrt(m/nk)
         *   飞镖拉力所需距离
        */ 
        s = m.velocity * std::sqrt(this->mass / (this->n * this->k_s));
        m.s = s;  // 飞镖所需拉力的距离

        vel_pub_->publish(m);
    };

}
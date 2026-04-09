/*一些会用到的函数（格式转化及画图函数）*/

#ifndef COMMON_HPP_
#define COMMON_HPP_

#ifdef _WIN32
#include <process.h>
#define GETPID _getpid
#else
#include <unistd.h>
#define GETPID getpid
#endif

#include <opencv2/opencv.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <string>

int encoding2mat_type(const std::string & encoding)
{
    if(encoding == "mono8") {
        return CV_8UC1;
    } else if(encoding == "bgr8") {
        return CV_8UC3;
    } else if(encoding == "mono16") {
        return CV_16SC1;
    } else if(encoding == "rgba8") {
        return CV_8UC4;
    }
    throw std::runtime_error("是OpenCV不支持的Mat格式呢");
}

std::string mat_type2encoding(int mat_type)
{
    switch(mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("是OpenCV不支持的编码格式呢");
    }
}

void set_now(builtin_interfaces::msg::Time & time)
{
    std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
    if(now <= std::chrono::nanoseconds(0)) {
        time.sec = time.nanosec = 0;
    } else {
        time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
        time.nanosec = now.count() % 1000000000;
    }
}

#endif
#ifndef __MINDVISION_CAMERA_HPP__
#define __MINDVISION_CAMERA_HPP__

#include <CameraApi.h>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <iostream>
#include <unordered_map>
#include <mutex>

#include "termial_color.hpp"

#define MINDVISION_CAMERA_NUMBER 4  // 将会接入多少个迈德威视相机，每台机器人配置时修改

using std::unordered_map;
using std::string;
using std::cout;
using std::endl;

// 用于状态检查的宏
#define CHECK if(process_state != 0) throw std::runtime_error(analyse_state(process_state));

namespace camera
{

class Mindvision
{
public:
    Mindvision();
    ~Mindvision();
protected:
    int process_state = -1;     // 指示迈德威视相机模块程序的状态
    tSdkCameraDevInfo info_list[MINDVISION_CAMERA_NUMBER];    // 迈德威视相机列表，类型来自SDK定义
    int camera_amount;      // 相机的数量
    struct cam  // 迈德威视相机配置所需数据的包装
    {
        int handle;     // 相机句柄
        struct size {
            int width;
            int height;
            int Woffset;
            int Hoffset;
        }size;
        struct exposure {
            double time;    // 单位：微妙
            bool auto_enable;
            int auto_target;
        }exposure;
        struct trigger_set {
            bool soft_enable;
            double fps;
        }trigger;
        struct img {    // 图像数据
            tSdkFrameHead  sFrameInfo;  // 图像帧头信息
            unsigned char *pRawBuffer;  // 原始格式图像缓冲区
            unsigned char *pRbgBuffer;  // 
        }img;
    };
    unordered_map<string, cam> cam_;     // 调用迈德威视相机所需的一切“名字”，这里的键使用相机的设备昵称
    std::mutex cam_mutex_;      // 为cam_准备的锁
    
    cv::Mat GetImage(const std::string & name);     // 把通过SDK得到的相机图像转换成opencv矩阵格式

private:
    void Init();        // 依据配置文件开启相机
    std::string analyse_state(int state);   // 用于检查状态的宏
};
}

#endif
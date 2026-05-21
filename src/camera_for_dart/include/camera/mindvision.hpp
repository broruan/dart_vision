#ifndef MINDVISION_HPP
#define MINDVISION_HPP

#include <CameraApi.h>
#include <atomic>

#include "opencv4/opencv2/core/core.hpp"

namespace sensor {

class MindVision {
public:
    explicit MindVision(std::string mindvision_config = "", const std::string sn = "");
    ~MindVision();

    /**
     * @brief 用于飞镖的软触发模式
     */

     void SoftTriggerEnable() {
        CameraSetTriggerCount(h_camera, 1);// 一次触发两帧
        CameraSetTriggerMode(h_camera, 1); // 软触发
     }

     void SoftTrigger() {
        CameraSoftTrigger(h_camera);
     }

    /**
     * @brief 获取图像
     * @param frame 用于保存图像的 std::shared_ptr<cv::Mat>
     * @return bool 是否成功获取图像
     */
    bool GetFrame(std::shared_ptr<cv::Mat>& frame);

    bool GetCameraStatus() {
        return (i_camera_counts != 0) && (i_status == CAMERA_STATUS_SUCCESS);
    }

    void SetExposureTime(int time) {
        CameraSetExposureTime(h_camera, time);
    }

    void GetExposureTime(double &time) {
        CameraGetExposureTime(h_camera, &time);
    }

    void GetWbMode(int &mode) {
        CameraGetWbMode(h_camera,&mode);
    }

    int SetOnceWB() {
        return CameraSetOnceWB(h_camera);
    }

    void GetExposureTimeRange(double &min, double &max, double &step) {
        CameraGetExposureTimeRange(h_camera, &min, &max, &step);
    }

    int Overlay(){
        return CameraImageOverlay(h_camera, pby_buffer, &s_frame_info);
    }

    void GetClrTempMode(int &mode) {
        CameraGetClrTempMode(h_camera, &mode);
    }

    int SetClrTempMode(int mode) {
        return CameraSetClrTempMode(h_camera, mode);
    }

    void SetExposureTime(std::string mindvision_config) {
        CameraReadParameterFromFile(h_camera, mindvision_config.data());
    }
    void SetGain(int gain) {
        CameraSetGain(h_camera, gain, gain, gain);
    }

private:
    // 相机数量
    int i_camera_counts;
    // 相机状态，在 CameraStatus.h 中查看
    int i_status;

    // 相机其他信息
    tSdkCameraDevInfo t_camera_enum_list[2];
    int h_camera;
    tSdkCameraCapbility t_capability;
    tSdkFrameHead s_frame_info;
    BYTE* pby_buffer;
    unsigned char* g_p_rgb_buffer; // 处理后数据缓存区
    
};

} // namespace sensor

#endif // MINDVISION_HPP


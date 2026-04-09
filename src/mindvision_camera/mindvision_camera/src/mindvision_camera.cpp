#include "mindvision_camera/mindvision_camera.hpp"

namespace camera
{

Mindvision::Mindvision() :
camera_amount(MINDVISION_CAMERA_NUMBER)
{
    // 这个Init()检测所有连上电脑的相机的昵称
    Init();
}

Mindvision::~Mindvision()
{
    for (const auto & cam : cam_) {
        CameraUnInit(cam.second.handle);
        delete[] cam.second.img.pRbgBuffer;
    }
}

void Mindvision::Init()
{
    // 开始MV相机配置
    // 套上CHECK后处理
    // 死了都要TRY，这里的TRY合理吗？
    try{
        // 使能SDK，之后才能够调用SDK接口
        process_state = CameraSdkInit(1);CHECK
        // 枚举连上电脑的相机设备，这之后得到的camera_amount是连上电脑的实际相机数量
        process_state = CameraEnumerateDevice(info_list, &camera_amount);CHECK
        #if 0
            if (camera_amount != MINDVISION_CAMERA_NUMBER) {
                cout << YELLOW << "程序实际连上的相机数量：" << camera_amount << endl;
                cout << "与宏定义数目不匹配，宏定义数目：" << MINDVISION_CAMERA_NUMBER << RESET << endl;
                std::flush(std::cout);
            } else {
                cout << "程序实际连上的相机数量：" << camera_amount << endl;
                std::flush(std::cout);
            }
        #endif
        // 遍历设备列表
        for(int i = 0; i < camera_amount; ++i) {
            // 相机初始化
            CameraHandle temp_handle;
            process_state = CameraInit(&info_list[i], emSdkParameterMode::PARAM_MODE_BY_NAME, emSdkParameterTeam::PARAMETER_TEAM_B, &temp_handle);CHECK
            // 查看设备昵称
            #if 0
                cout << "设备列表的第 " << i+1 << " 个相机：" << endl; 
            #endif
            char temp_name[32];
            process_state = CameraGetFriendlyName(temp_handle, temp_name); CHECK
            std::string name(std::move(temp_name));
            cam_[name].handle = std::move(temp_handle);
            std::string File = "Camera/Configs/"+ name +"-Group1.config";
            if (CameraReadParameterFromFile(cam_[name].handle, File.data()) != 0) {
                //TODO: 改成型号默认
                cout << "使用默认参数文件" << endl;
                File = "Camera/Configs/Default.config";
                process_state = CameraReadParameterFromFile(cam_[name].handle, File.data()); CHECK
            }
            #if 0
                cout << "昵称为：" << name << endl;
            #endif

            tSdkCameraCapbility temp_cap;
            CameraGetCapability(cam_[name].handle, &temp_cap);
            //TODO: 可选配置显示更多的信息
            //TODO: 之后适配更多Mindvision的相机型号，这里会需要更复杂的逻辑
            // 设置图像处理输出格式
            if (temp_cap.sIspCapacity.bMonoSensor) {
                CameraSetIspOutFormat(cam_[name].handle, CAMERA_MEDIA_TYPE_MONO8);
            } else {
                CameraSetIspOutFormat(cam_[name].handle, CAMERA_MEDIA_TYPE_BGR8);
            }
            // 设置图像缓冲区
            cam_[name].img.pRbgBuffer = new unsigned char [
                temp_cap.sResolutionRange.iHeightMax * temp_cap.sResolutionRange.iWidthMax * 3
            ];
            
            // 把硬件设备初始化的出的参数设置到成员变量cam_里
            // 调用 SDK 的各种函数，看看手册
            tSdkImageResolution temp_img_resolution;
            process_state = CameraGetImageResolution(cam_[name].handle, &temp_img_resolution); CHECK
            //TODO: 这里这个参数需要确认修改，初衷是要做最终图像的大小限制
            cam_[name].size.height = temp_img_resolution.iHeight;
            cam_[name].size.width = temp_img_resolution.iWidth;
            cam_[name].size.Hoffset = temp_img_resolution.iHOffsetFOV;
            cam_[name].size.Woffset = temp_img_resolution.iVOffsetFOV;
            process_state = CameraGetExposureTime(cam_[name].handle, &cam_[name].exposure.time); CHECK
            int temp_ae_state = 0;
            process_state = CameraGetAeState(cam_[name].handle, &temp_ae_state); CHECK
            cam_[name].exposure.auto_enable = static_cast<bool>(temp_ae_state);
            process_state = CameraGetAeTarget(cam_[name].handle, &cam_[name].exposure.auto_target); CHECK
            //TODO: 添加硬触发处理
            int temp_tri_mode = 0;
            process_state = CameraGetTriggerMode(cam_[name].handle, &temp_tri_mode); CHECK
            cam_[name].trigger.soft_enable = (0 == temp_tri_mode)? false:true;

            #if 0
                cout << BLUE
                     << "相机名：" << name << "\n"
                     << "图像宽度：" << cam_[name].size.width << "\n"
                     << "图像高度：" << cam_[name].size.height << "\n"
                     << "自动曝光使能：" << cam_[name].exposure.auto_enable << "\n"
                     << "自动曝光目标值：" << cam_[name].exposure.auto_target << "\n"
                     << "定曝光时间：" << cam_[name].exposure.time << "\n"
                     << "触发采集帧率：" << cam_[name].trigger.fps << "\n"
                     << "触发使能：" << cam_[name].trigger.soft_enable << RESET << endl;
            #endif

            // 启动该相机
            CameraPlay(cam_[name].handle);
        }

        }
    catch(const std::runtime_error& err){
        std::cout << RED << err.what() << std::endl;
    }
}

cv::Mat Mindvision::GetImage(const std::string & name)
{
    //TODO: 检查是否会有丢帧、连帧问题
    process_state = CameraGetImageBuffer(cam_[name].handle, &cam_[name].img.sFrameInfo, &cam_[name].img.pRawBuffer, 1000); CHECK

    process_state = CameraImageProcess(cam_[name].handle, cam_[name].img.pRawBuffer, cam_[name].img.pRbgBuffer, &cam_[name].img.sFrameInfo); CHECK
    cv::Mat result(
        cv::Size(cam_[name].img.sFrameInfo.iWidth, cam_[name].img.sFrameInfo.iHeight),
        cam_[name].img.sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
        cam_[name].img.pRbgBuffer
    );
    process_state = CameraReleaseImageBuffer(cam_[name].handle, cam_[name].img.pRawBuffer); CHECK

    return result;
}

std::string Mindvision::analyse_state(int state)
{
    switch (state) {
    case CAMERA_STATUS_SUCCESS:                 return "操作成功";
    case CAMERA_STATUS_FAILED:                  return "操作失败";
    case CAMERA_STATUS_INTERNAL_ERROR:          return "内部错误";
    case CAMERA_STATUS_UNKNOW:                  return "未知错误";
    case CAMERA_STATUS_NOT_SUPPORTED:           return "不支持该功能";
    case CAMERA_STATUS_NOT_INITIALIZED:         return "初始化未完成";
    case CAMERA_STATUS_PARAMETER_INVALID:       return "参数无效";
    case CAMERA_STATUS_PARAMETER_OUT_OF_BOUND:  return "参数越界";
    case CAMERA_STATUS_UNENABLED:               return "未使能";
    case CAMERA_STATUS_USER_CANCEL:             return "用户手动取消了，比如roi面板点击取消，返回";
    case CAMERA_STATUS_PATH_NOT_FOUND:          return "注册表中没有找到对应的路径";
    case CAMERA_STATUS_SIZE_DISMATCH:           return "获得图像数据长度和定义的尺寸不匹配";
    case CAMERA_STATUS_TIME_OUT:                return "超时错误";
    case CAMERA_STATUS_IO_ERROR:                return "硬件IO错误";
    case CAMERA_STATUS_COMM_ERROR:              return "通讯错误";
    case CAMERA_STATUS_BUS_ERROR:               return "总线错误";
    case CAMERA_STATUS_NO_DEVICE_FOUND:         return "没有发现设备";
    case CAMERA_STATUS_NO_LOGIC_DEVICE_FOUND:   return "未找到逻辑设备";
    case CAMERA_STATUS_DEVICE_IS_OPENED:        return "设备已经打开";
    case CAMERA_STATUS_DEVICE_IS_CLOSED:        return "设备已经关闭";
    case CAMERA_STATUS_DEVICE_VEDIO_CLOSED:     return "没有打开设备视频，调用录像相关的函数时，如果相机视频没有打开，则回返回该错误。";
    case CAMERA_STATUS_NO_MEMORY:               return "没有足够系统内存";
    case CAMERA_STATUS_FILE_CREATE_FAILED:      return "创建文件失败";
    case CAMERA_STATUS_FILE_INVALID:            return "文件格式无效";
    case CAMERA_STATUS_WRITE_PROTECTED:         return "写保护，不可写";
    case CAMERA_STATUS_GRAB_FAILED:             return "数据采集失败";
    case CAMERA_STATUS_LOST_DATA:               return "数据丢失，不完整";
    case CAMERA_STATUS_EOF_ERROR:               return "未接收到帧结束符";
    case CAMERA_STATUS_BUSY:                    return "正忙(上一次操作还在进行中)，此次操作不能进行";
    case CAMERA_STATUS_WAIT:                    return "需要等待(进行操作的条件不成立)，可以再次尝试trf";
    case CAMERA_STATUS_IN_PROCESS:              return "正在进行，已经被操作过";
    default:                                    return "不在常见错误码里，错误码："+std::to_string(state);
    }
}

}
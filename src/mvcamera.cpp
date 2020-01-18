/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.6.30

 Update: Wang Xiaoyan on 2019.10.14

 Detail: 实现对工业摄像头的基本参数设定和初始化，并将当前帧的数据转化为opencv的Mat类型。
         具体可参考MindVision的官方linux SDK，这里附上SDK的下载地址:
         http://www.mindvision.com.cn/rjxz/list_12.aspx?lcid=138
 *****************************************************************************/

#include "mvcamera.h"
#include <iostream>

MVCamera::MVCamera()
{
    is_open_ = false;
}

MVCamera::~MVCamera()
{
    close();
}

void MVCamera::open(int frame_width,
                    int frame_height,
                    int exposure_time,
                    int frame_speed,
                    int gamma,
                    int contrast)
{
    int status = -1;
    int cameraCounts = 1;
    int channel = 3;

    CameraSdkInit(0);

    // 枚举设备，并建立设备列表
    status = CameraEnumerateDevice(&tCameraEnumList, &cameraCounts);
    if (cameraCounts == 0)
    {
        throw MVCameraException("Enumerate camera failed.");
    }

    /*int i = 0;
    for (; i < CAMERA_COUNT; ++i)
    {
        if (eqString("049080710034", tCameraEnumList[i].acSn, 13))
            break;
    }

    if (i == CAMERA_COUNT)
    {
        throw MVCameraException("Cannot find valid acSn");
    }*/
    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    status = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    if (status != CAMERA_STATUS_SUCCESS)
    {
        throw MVCameraException("Init camera failed.");
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);
    g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iWidthMax * tCapability.sResolutionRange.iHeightMax * 3);
    // 让SDK进入工作模式，开始接收来自相机发送的图像数据。如果当前相机是触发模式，则需要接收到触发帧以后才会更新图像
    CameraPlay(hCamera);

    tSdkImageResolution imageResolution;
    CameraGetImageResolution(hCamera, &imageResolution);
    imageResolution.iIndex = 0XFF;
    imageResolution.iWidth = frame_width;
    imageResolution.iHeight = frame_height;
    CameraSetImageResolution(hCamera, &imageResolution); // 设置图像的分辨率
    CameraSetAeState(hCamera, 0);                        // 设置为手动曝光
    CameraSetExposureTime(hCamera, exposure_time);       // 设置曝光
    CameraSetFrameSpeed(hCamera, frame_speed);           // 设置帧率
    CameraSetGamma(hCamera, gamma);                      // 设置gamma
    CameraSetContrast(hCamera, contrast);                // 设置对比度

    if (tCapability.sIspCapacity.bMonoSensor)
    {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8); // 8位单通道
    }
    else
    {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8); // 三通道
    }
    is_open_ = true;
}

bool MVCamera::isOpen()
{
    return is_open_;
}

void MVCamera::close()
{
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);
    is_open_ = false;
}

void MVCamera::getImage(cv::Mat &image)
{
    if (!is_open_)
    {
        throw MVCameraException("Get image error. Camera is not opened.");
    }

    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 500) == CAMERA_STATUS_SUCCESS)
    {

        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

        image = cv::Mat(cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                        CV_8UC3,
                        g_pRgbBuffer);

        // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer
        // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
    }
    else
    {
        throw MVCameraException("Get image failed. Camera is not opened.");
    }
}

cv::Mat MVCamera::getImage()
{
    if (!is_open_)
    {
        throw MVCameraException("Get image error. Camera is not opened.");
    }

    cv::Mat image;
    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 500) == CAMERA_STATUS_SUCCESS)
    {

        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

        image = cv::Mat(cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                        CV_8UC3,
                        g_pRgbBuffer);

        // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer
        // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
    }
    else
    {
        throw MVCameraException("Get image failed. Camera is not opened.");
    }

    return image;
}

bool MVCamera::eqString(const char *s1, const char *s2, int len)
{
    int i = 0;
    for (; i < len; ++i)
    {
        if (!(s1[i] == s2[i]))
            break;
    }

    return i == len;
}

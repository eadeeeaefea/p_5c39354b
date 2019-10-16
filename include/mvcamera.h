/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.6.30

 Update: Wang Xiaoyan on 2019.10.14

 Detail: 工业摄像头调用相关。实现对工业摄像头的基本参数设定和其初始化。
 *****************************************************************************/

#ifndef HERORM2020_MVCAMERA_H
#define HERORM2020_MVCAMERA_H

#include <cstring>
#include <exception>
#include <opencv2/core/core.hpp>
#include "CameraApi.h"

using std::string;
using std::exception;


class MVCamera {
private:
    unsigned char*      g_pRgbBuffer;     // 处理后图像输出的缓冲区
    int                 hCamera;          // 相机的句柄
    tSdkCameraDevInfo   tCameraEnumList;  // 工业摄像头设备列表数组
    tSdkCameraCapbility tCapability;      // 相机特性描述的结构体
    tSdkFrameHead       sFrameInfo;       // 图像的帧头信息
    unsigned char*      pbyBuffer;        // 指向图像的数据的缓冲区

    bool is_open_;

public:
    MVCamera();
    ~MVCamera();

    void open(int frame_width = 640,
              int frame_height = 480,     // MV-SUA33GC-T的最大分辨率为640*480, 其他分辨率为ROI后的结果
              int exposure_time = 12000,  // unit: us  range: [2, 20700]
              int frame_speed = 2,        // 0: low  1: normal  2: high
              int gamma = 100,            // 默认100
              int contrast = 100);        // 默认100
    bool isOpen();
    void close();

    void getImage(cv::Mat &image);
    cv::Mat getImage();

};

class MVCameraException : public exception {
private:
    string e_what_;

public:
    MVCameraException() {}
    MVCameraException(const string &error) : e_what_(error) {}
    virtual ~MVCameraException() throw() {}
    virtual const char *what() const throw() {
        return e_what_.c_str();
    }

};


#endif  // HERORM2020_MVCAMERA_H

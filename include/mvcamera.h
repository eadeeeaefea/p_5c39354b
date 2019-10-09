/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.6.30

 Update: Wang Xiaoyan on 2019.10.8

 Detail: 工业摄像头调用相关。实现对工业摄像头的基本参数设定和其初始化。
 *****************************************************************************/

#ifndef HERORM2020_MVCAMERA_H
#define HERORM2020_MVCAMERA_H

#include "opencv2/core/core.hpp"
#include "CameraApi.h"


class MVCamera {
private:
    enum Status {
        STOPPED,
        RUNNING
    };

    unsigned char*      g_pRgbBuffer;     // 处理后图像输出的缓冲区
    int                 hCamera;          // 相机的句柄
    tSdkCameraDevInfo   tCameraEnumList;  // 工业摄像头设备列表数组
    tSdkCameraCapbility tCapability;      // 相机特性描述的结构体
    tSdkFrameHead       sFrameInfo;       // 图像的帧头信息
    unsigned char*      pbyBuffer;        // 指向图像的数据的缓冲区

    bool status_;

public:
    /**
     * @breif: 构造函数
     * @param: None
     * @return: None
     */
    MVCamera();

    /**
     * @breif: 析构函数
     * @param: None
     * @return: None
     */
    ~MVCamera();

    /**
     * @breif: 打开工业摄像头并进行相应的初始化和参数的赋值
     * @param: MV-SUA33GC-T的最大分辨率为640*480, 其他分辨率为ROI后的结果
     *         exposure_time: unit: us  range: [2, 20700]
     *         frame_speed: 0: low  1: normal  2: high
     *         gamma: 默认100
     *         contrast: 默认100
     * @return: None
     */
    void open(int frame_width = 640,
              int frame_height = 480,
              int exposure_time = 12000,
              int frame_speed = 2,
              int gamma = 100,
              int contrast = 100);

    /**
     * @breif: 关闭工业摄像头
     * @param: None
     * @return: None
     */
    void close();

    /**
     * @breif: 从摄像头读取Mat类型的图片
     * @param: image 读取到的图片
     * @return: None
     */
    void getImage(cv::Mat &image);

    /**
     * @breif: 从摄像头读取Mat类型的图片
     * @param: None
     * @return: 读取到的图片
     */
    cv::Mat getImage();

};


#endif  // HERORM2020_MVCAMERA_H

/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_BASE_H
#define HERORM2020_BASE_H

//显示各模块运行时间
#define RUNNING_TIME

//选择使用的颜色空间 BGR/HSV
#define BGR
#ifndef BGR
#define HSV
#endif

//显示图片
#define SHOW_IMAGE

//显示图片时,同时显示TrackBar
#define TRACKBAR
#if (defined(TRACKBAR) && !defined(SHOW_IMAGE))
#define SHOW_IMAGE
#endif

//导入相机内参矩形和畸变系数
#define DISTORTION_CORRECT

//使用ROI来加速
#define ROI_ENABLE

//使用摄像头
#define USE_CAMERA

//使用串口
#define USE_SERIAL

//Qt上位机画显示数据
//#define PLOT_DATA

//使用本地文件进行测试 1-图片 2-视频
#if (!defined(USE_CAMERA) && !defined(USE_SERIAL))
#define TEST 2
#define IMAGE_PATH "source.jpg"
#define VIDEO_PATH "source.avi"
#endif

//保存视频流 1-放进缓冲区使用 2-显示但不进行处理
//#define SAVE_VIDEO 1
#if defined(SAVE_VIDEO) && !defined(USE_CAMERA)
#define USE_CAMERA
#endif

//视频保存路径
#define VIDEO_SAVED_PATH "output.avi"

//手动决定目标颜色,比赛时必须将其注释
#define ENEMY_COLOR 0

//保持装甲板模式
#define ARMOR_ONLY

//常量定义
#define PI 3.141592654
#define g 9.7988

//相机配置
#define FRAME_WIDTH   640
#define FRAME_HEIGHT  480
#define EXPOSURE_TIME 800

//配置文件路径
#define PARAM_PATH "../param/param.xml"

typedef struct 
{
    int mode;
    double yaw;
    double pitch;
}SendPack;

typedef struct 
{
    int enemy_color;  // 0-red, 1-blue
    int mode;
    double pitch;
    double yaw;
}ReadPack;

typedef struct
{
    int plot_type;
    int curve_num;
    double plot_value[3];
}PlotPack;

//模式,只用于接收数据包
enum Mode 
{
    ARMOR = 0x50,
    RUNE,
    STAND_BY
};

#endif  // HERORM2020_BASE_H

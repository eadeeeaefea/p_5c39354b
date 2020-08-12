/**
 * @file workspace.h
 * @brief 
 * @details 
 * @author Li Haotian on 2020-05-09
 * @email lcyxlihaotian@126.com
 * @update 
 * @version 
 * @license 2020 HITWH HERO-Robomaster Group
 */
#ifndef HERORM2020_WORKSPACE_H
#define HERORM2020_WORKSPACE_H

#include <unistd.h>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/tracking.hpp>

#include "robotdetector.h"
#include "robottracker.h"
#include "armorchecker.h"
#include "serialport.h"
#include "dhcamera.h"
#include "base.h"
#include "timer.h"

using namespace cv;
using namespace std;

class Workspace
{
private:
    // image buffer used
    mutex image_buffer_metex; // 图像接受线程互斥锁
    int max_image_buffer_size_ = 10;
    // serial communication used
    SerialPort serial_port; // 与电控通信串口类
    SerialPort plot_serial; // 向上位机发送数据串口
    SendPack send_pack_;    // 向裁判系统发送数据包
    ReadPack read_pack_;    // 从裁判系统接受数据包
    PlotPack plot_pack_;    // 向上位机发送数据包

    // detect enemy robots used
    RobotDetector robot_detector;    // 前景提取查找机器人的类
    RobotTracker robot_tracker;      // 跟踪机器人的类
    Ptr<MultiTracker> enemy_tracker; // OpenCV多个跟踪器容器
    ArmorChecker armor_checker;      // 查找机器人装甲板并截取装甲板区域ROI图片的类
    DHCamera dh_camera;

    // knn背景建模算法事先需要提取的视频帧数（该参数需在robotdetector头文件中同步修改同名变量)
    int history_num_ = 10;
    // 为了方便查看设置的缩放视频图片大小的参数（这个参数的修改会导致后面其他参数的修改）
    int image_ratio_ = 2;
    // 检测稳定帧数的阈值（待调参）(robottracker)
    int check_thres_ = 5;
    //检测计数（计算检测稳定帧数和跟踪帧数）
    int check_num_ = 0;

    vector<Mat> robot_image_buffer_;
    Mat current_frame_; // 当前处理的帧

    // detect enemy missile used (remained)
    /*
        ...    
    */

public:
    Workspace();
    ~Workspace();
    void init();
    void run();

private:
    void imgRecvFunc();    // 图像接受线程
    void robImgProcFunc(); // 机器人检测图像处理线程
    void misImgProcFunc(); // 飞镖检测图像处理线程
    void msgCommFunc();    // 通信线程
    void openSerialPort(); // 打开串口，防止异常
};

#endif // HERORM2020_WORKSPACE_H
/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author:  Bruce Hou

 Detail:
 *****************************************************************************/
#ifndef HERORM2020_PREDICT_H
#define HERORM2020_PREDICT_H

#include<cmath>
#include<iostream>
#include <opencv2/opencv.hpp>
#include "base.h"
#include "anglesolver.h"

using namespace std;
using namespace cv;

class Predict {
private:
    int count = 0;

    VideoWriter writer; // 保存视频
    AngleSolver anglesolver; // 弹道角度求解
    vector<Point2f> object; // 云台绝对角度队列
    vector<Point2f> d_object; // 储存云台绝对角度差值
    Vec4f predict_line; // 线性预测
    Point2f object_motion;
    Point2f predict_object_motion;
    Point2f piline, vect; // 预测直线上的一点和方向向量

    double time_for_excercise; // 打击时间
    vector<double> flight_time; // 打击时间队列
    double sum_pitch, sum_yaw; // 绝对pitch和绝对yaw
//#ifdef SHOW_IMAGE
//    vector<Point2f>ap;
//#endif
public:
    Predict();

    ~Predict();

    void init();

    // 调用接口，read_pitch和read_yaw为获得图像的云台角度，readpitch和readyaw为发送的云台角度
    void run(double &x, double &y, double &z, double v, double &send_pitch, double &send_yaw, double read_pitch,
             double read_yaw, double readpitch, double readyaw); // 通过获取图像时的云台坐标，以及计算完成后的云台坐标，计算pitch和yaw


private:
    void update(); // 更新队列

    void motion_prediction(); // 位置预测

    bool judgement(); // 判断是否满足预测条件

    void coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    void anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    double point_distance(Point3d point1, Point3d point2);

    void get_excercise_time(double x, double y, double v, double ptz_pitch);
};


#endif  // HERORM2020_PREDICT_H
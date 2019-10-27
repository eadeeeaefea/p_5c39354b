/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Ma Yuemeng & Song Haodi

 Update: Wang Xiaoyan on 2019.7.1
         Zeng Jing on 2019.10.27.

 Detail:弹道计算
 *****************************************************************************/

#ifndef HERORM2020_ANGLESOLVER_H
#define HERORM2020_ANGLESOLVER_H

#include <cmath>
#include <iostream>
#include "base.h"
#ifdef RUNNING_TIME
#include "timer.h"
#endif
//#define DEBUG


class AngleSolver {
private:
    double yaw_offset_, pitch_offset_,g;

public:
    AngleSolver();
    ~AngleSolver();
    void init();

    //弹道计算，根据坐标值计算出云台需调整的角度值，输入坐标值和初速度、云台绝对pitch角，返回pitch角和yaw角
    void newrun(double x, double y, double z, double v,double& yaw, double& pitch, double ptz_pitch);
    //将坐标转化回标准坐标系，然后二分法计算角度
    void newdichotomy(double x, double y, double z, double v,double& yaw, double& pitch);//无坐标转化的二分法
    void speedtriangle(double x, double y, double z, double v,double& yaw, double& pitch);
    //无坐标转化的有速度矢量三角推导出的公式实现
    void run(double x, double y, double z, double v, double &yaw, double &pitch);//上届算法

    double get_yaw_offset();
    double get_pitch_offset();
    void set_yaw_offset(double yaw_offset);
    void set_pitch_offset(double pitch_offset);

private:
    bool parabolaSolve(double x, double y, double v, double &theta);
    double parabolaDeltaY(double x, double y, double v, double theta);
};

#endif  // HERORM2020_ANGLESOLVER_H

/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Ma Yuemeng & Song Haodi

 Update: Wang Xiaoyan on 2019.7.1

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_ANGLESOLVER_H
#define HERORM2020_ANGLESOLVER_H

#include <cmath>
#include <iostream>
#include "base.h"
#include "timer.h"
#ifdef RUNNING_TIME
#include "timer.h"
#endif

class AngleSolver {
private:
    double yaw_offset_, pitch_offset_,g;
    double origin_pitch;
    double origin_yaw;

public:
    AngleSolver();
    ~AngleSolver();
    void init();
    void run(double x, double y, double z, double v, double &yaw, double &pitch, double ptz_pitch);

    void setOriginPitch(double pitch);
    void setOriginYaw(double yaw);
    double getOriginPitch();
    double getOriginYaw();

    double get_yaw_offset();
    double get_pitch_offset();
    void set_yaw_offset(double yaw_offset);
    void set_pitch_offset(double pitch_offset);

    //其他备用算法
    //转换坐标系加二分法计算函数
    void change_dichotomy(double x, double y, double z, double v,double& yaw, double& pitch, double ptz_pitch);
    // 未加坐标旋转变换，稍微慢一些的两个算法，速度矢量三角计算快于二分法。
    //二分法算法原理同上
    void dichotomy(double x, double y, double z, double v,double& yaw, double& pitch);
    void speedtriangle(double x, double y, double z, double v,double& yaw, double& pitch);
private:
    bool parabolaSolve(double x, double y, double v, double &theta, double ptz_pitch);

};


#endif  // HERORM2020_ANGLESOLVER_H


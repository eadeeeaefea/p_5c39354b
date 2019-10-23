/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Ma Yuemeng & Song Haodi

 Update: Wang Xiaoyan on 2019.7.1.
　　　　 Zeng Jing on 2019.10.21.
 Detail:
 *****************************************************************************/

#ifndef HERORM2020_ANGLESOLVER_H
#define HERORM2020_ANGLESOLVER_H

#include <cmath>
#include <iostream>
#include "base.h"
#ifdef RUNNING_TIME
#include "timer.h"
#endif


class AngleSolver {
private:
    double yaw_offset_, pitch_offset_,g;

public:
    AngleSolver();
    ~AngleSolver();
    void init();

    void newrun(double x, double y, double z, double v,double& yaw, double& pitch, double ptz_pitch);
    void newdichotomy(double x, double y, double z, double v,double& yaw, double& pitch);
    void speedtriangle(double x, double y, double z, double v,double& yaw, double& pitch);
    void run(double x, double y, double z, double v, double &yaw, double &pitch);
    double get_yaw_offset();
    double get_pitch_offset();
    void set_yaw_offset(double yaw_offset);
    void set_pitch_offset(double pitch_offset);

private:
    bool parabolaSolve(double x, double y, double v, double &theta);
    double parabolaDeltaY(double x, double y, double v, double theta);

};


#endif  // HERORM2020_ANGLESOLVER_H

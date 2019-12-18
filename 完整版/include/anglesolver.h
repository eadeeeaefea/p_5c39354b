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

class AngleSolver {
private:
    double yaw_offset_, pitch_offset_;
    double origin_pitch;
    double origin_yaw;
public:
    AngleSolver();
    ~AngleSolver();
    void setOriginPitch(double pitch);
    void setOriginYaw(double yaw);
    double getOriginPitch();
    double getOriginYaw();
    void init();
    void run(double x, double y, double z, double v, double &yaw, double &pitch, double ptz_pitch);
    double get_yaw_offset();
    double get_pitch_offset();
    void set_yaw_offset(double yaw_offset);
    void set_pitch_offset(double pitch_offset);

private:
    bool parabolaSolve(double x, double y, double v, double &theta, double ptz_pitch);
    double parabolaDeltaY(double x, double y, double v, double theta);

};


#endif  // HERORM2020_ANGLESOLVER_H
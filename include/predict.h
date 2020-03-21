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

    VideoWriter writer;
    AngleSolver anglesolver;
    vector<Point2f> object;
    vector<Point2f> d_object;
    Vec4f predict_line;
    Point2f object_motion;
    Point2f predict_object_motion;
    Point2f piline, vect;

    double time_for_excercise;
    double sum_pitch, sum_yaw;
public:
    Predict();

    ~Predict();

    void init();

    void run(double &x, double &y, double &z, double v, double &send_pitch, double &send_yaw, double read_pitch,
             double read_yaw, double readpitch, double readyaw);

private:
    void update();

    void motion_prediction();

    bool judgement();

    void coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    void anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    double point_distance(Point3d point1, Point3d point2);

    void get_excercise_time(double x, double y, double v, double ptz_pitch);
};


#endif  // HERORM2020_PREDICT_H
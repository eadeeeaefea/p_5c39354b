/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.11.5

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_PREDICTOR_H
#define HERORM2020_PREDICTOR_H

#include "iostream"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class Predictor {
private:
    vector<Point3d> object_coordinate;
    vector<Point3d> object_inaccurate_speed;
    Point3d object_accurate_speed;
    Point3d object_motion;
public:
    Predictor();

    ~Predictor();

    void init();

    void run(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

private:
    void motion_prediction(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    void coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    void anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    double point_distance(Point3d point1, Point3d point2);

    bool judgement();
};


#endif  // HERORM2020_PREDICTOR_H

/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Bruce Hou
 Update:
 Detail:
 *****************************************************************************/
#include "generalfunction.h"

void coordinate_transformation(double &x, double &y, double &z,
                               double ptz_pitch, double ptz_yaw) {
    double absolute_x;
    double absolute_y;
    double absolute_z;
    absolute_x = x * cos(ptz_yaw) - y * sin(ptz_pitch) * sin(ptz_yaw) -
                 z * cos(ptz_pitch) * sin(ptz_yaw);
    absolute_y = y * cos(ptz_pitch) - z * sin(ptz_pitch);
    absolute_z = x * sin(ptz_yaw) + y * sin(ptz_pitch) * cos(ptz_yaw) +
                 z * cos(ptz_pitch) * cos(ptz_yaw);
    x = absolute_x;
    y = absolute_y;
    z = absolute_z;
}

void anti_coordinate_transformation(double &x, double &y, double &z,
                                    double ptz_pitch,
                                    double ptz_yaw) {
    double objective_x;
    double objective_y;
    double objective_z;
    objective_x = x * cos(ptz_yaw) + z * sin(ptz_yaw);
    objective_y = -x * sin(ptz_yaw) * sin(ptz_pitch) + y * cos(ptz_pitch) +
                  z * cos(ptz_yaw) * sin(ptz_pitch);
    objective_z = -x * sin(ptz_yaw) * cos(ptz_pitch) - y * sin(ptz_pitch) +
                  z * cos(ptz_yaw) * cos(ptz_pitch);
    x = objective_x;
    y = objective_y;
    z = objective_z;
}

void draw(Mat &src, RotatedRect rect){
    Point2f pt[4];
    rect.points(pt);
    for(int i = 0; i < 3; i++){
        line(src, pt[i], pt[i+1], Scalar(0, 255, 0), 2);
    }
    line(src, pt[4], pt[0], Scalar(0, 255, 0), 2);
}

Rect get_roi(RotatedRect rect){
    Point2f pt[4];
    vector<Point2f> point;
    Rect roi;
    rect.points(pt);
    point.clear();
    for(int i=0; i<4; i++){
        point.emplace_back(pt[i]);
    }
    roi = boundingRect(point);
    return roi;
}
/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.11.5

 Detail:
 *****************************************************************************/

#include "predictor.h"
#include "base.h"
#include <cmath>
#include <iostream>

using std::cout;
using std::endl;


Predictor::Predictor() {

}

Predictor::~Predictor() {

}

void Predictor::init() {
}

void Predictor::run(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {
    motion_prediction(x, y, z, ptz_pitch, ptz_yaw);
}

void Predictor::motion_prediction(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {
    double angle_pitch, angle_yaw;
    angle_pitch = ptz_pitch * PI / 180;
    angle_yaw = ptz_yaw * PI / 180;
    int c;
    double dis;
    double time = 0.05;
    coordinate_transformation(x, y, z, angle_pitch, angle_yaw);
    object_motion = Point3d(x, y, z);
    if (object_coordinate.size() < 10) {
        object_coordinate.emplace_back(object_motion);
    } else {
        object_coordinate.erase(object_coordinate.begin());
        object_coordinate.emplace_back(object_motion);
    }
    dis = point_distance(object_coordinate[8], object_coordinate[9]);
    if (dis < 0.25 && dis > 0.001 && judgement()) {
        for (c = 0; c <= 4; c++) {
            object_inaccurate_speed[c] = (object_coordinate[c + 5] - object_coordinate[c]) / time;
        }
        object_accurate_speed = (object_inaccurate_speed[0] + object_inaccurate_speed[1] + object_inaccurate_speed[2] +
                                 object_inaccurate_speed[3] + object_inaccurate_speed[4]) / 5;
        x = object_coordinate[9].x + object_accurate_speed.x * 0.04;
        y = object_coordinate[9].y + object_accurate_speed.y * 0.04;
        z = object_coordinate[9].z + object_accurate_speed.z * 0.04;
        anti_coordinate_transformation(x, y, z, angle_pitch, angle_yaw);
    } else {
        anti_coordinate_transformation(x, y, z, angle_pitch, angle_yaw);
    }
}

void Predictor::coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {
    double absolute_x;
    double absolute_y;
    double absolute_z;
    absolute_x = x * cos(ptz_yaw) - y * sin(ptz_pitch) * sin(ptz_yaw) - z * cos(ptz_pitch) * sin(ptz_yaw);
    absolute_y = y * cos(ptz_pitch) - z * sin(ptz_pitch);
    absolute_z = x * sin(ptz_yaw) + y * sin(ptz_pitch) * cos(ptz_yaw) + z * cos(ptz_pitch) * cos(ptz_yaw);
    x = absolute_x;
    y = absolute_y;
    z = absolute_z;
}

void Predictor::anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {
    double absolute_x;
    double absolute_y;
    double absolute_z;
    absolute_x = x * cos(ptz_yaw) - y * sin(ptz_pitch) * sin(ptz_yaw) + z * cos(ptz_pitch) * sin(ptz_yaw);
    absolute_y = y * cos(ptz_pitch) + z * sin(ptz_pitch);
    absolute_z = -x * sin(ptz_yaw) - y * sin(ptz_pitch) * cos(ptz_yaw) + z * cos(ptz_pitch) * cos(ptz_yaw);
    x = absolute_x;
    y = absolute_y;
    z = absolute_z;
}

double Predictor::point_distance(Point3d point1, Point3d point2) {
    double distance;
    distance = sqrt(((point2.x - point1.x) * (point2.x - point1.x) + (point2.y - point1.y) * (point2.y - point1.y) +
                     (point2.z - point1.z) * (point2.z - point1.z)));
    return distance;
}

bool Predictor::judgement() {
    int i;
    double count = 0;
    for (i = 0; i < 10; i++) {
        if (object_coordinate[i] != Point3d(0, 0, 0)) {
            count += 1;
        }
    }
    if (count == 10) {
        return true;
    }
}

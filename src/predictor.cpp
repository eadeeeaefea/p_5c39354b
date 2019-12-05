/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.11.5
 Update: Bruce Hou on 2019.12.03
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
    get_excercise_time(sqrt(x * x + z * z), y, 20, ptz_pitch);
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
    if (dis < 0.25 && dis > 0.017 && judgement()) {
        for (c = 0; c <= 4; c++) {
            object_inaccurate_speed[c] = (object_coordinate[c + 5] - object_coordinate[c]) / time;
        }
        object_accurate_speed = (object_inaccurate_speed[0] + object_inaccurate_speed[1] + object_inaccurate_speed[2] +
                                 object_inaccurate_speed[3] + object_inaccurate_speed[4]) / 5;
        x = object_coordinate[9].x + object_accurate_speed.x * time_for_excercise;
        y = object_coordinate[9].y + object_accurate_speed.y * time_for_excercise;
        z = object_coordinate[9].z + object_accurate_speed.z * time_for_excercise;
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
    double objective_x;
    double objective_y;
    double objective_z;
    objective_x = x * cos(ptz_yaw) + z * sin(ptz_yaw);
    objective_y = -x * sin(ptz_yaw) * sin(ptz_pitch) + y * cos(ptz_pitch) + z * cos(ptz_yaw) * sin(ptz_pitch);
    objective_z = -x * sin(ptz_yaw) * cos(ptz_pitch) - y * sin(ptz_pitch) + z * cos(ptz_yaw) * cos(ptz_pitch);
    x = objective_x;
    y = objective_y;
    z = objective_z;
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

void Predictor::get_excercise_time(double x, double y, double v, double ptz_pitch) {
    double time_square;
    static const double g = 9.7988;
    double delta_angle;
    double x_bar;
    double y_bar;
    delta_angle = ptz_pitch * PI / 180;
    x_bar = x * cos(delta_angle) - y * sin(delta_angle);
    y_bar = x * sin(delta_angle) + y * cos(delta_angle);
    time_square =
            2.0 * ((y_bar * g + v * v) - sqrt(pow(g * y_bar + v * v, 2.0) - (x_bar * x_bar + y_bar * y_bar) * g * g)) /
            (g * g);
    time_for_excercise = sqrt(time_square) + 0.04;
    if (isnan(time_for_excercise)) {
        time_for_excercise = 0.04;
    }
}

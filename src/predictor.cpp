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
    delta_t = 0.12;
    x_last = 0.0;
    y_last = 0.0;
    z_last = 0.0;
}

void Predictor::run(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {
    motion_prediction(x, y, z, ptz_pitch, ptz_yaw);
}

void Predictor::motion_prediction(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {
    double angle_pitch, angle_yaw;
    angle_pitch = ptz_pitch * PI / 180;
    angle_yaw = ptz_yaw * PI / 180;
    int i;
    int c;
    double dis;
    double time = 0.05;
    coordinate_transformation(x, y, z, angle_pitch, angle_yaw);
    for(i=0; i<=8; i++){
        coordinate[i][0] = coordinate[i+1][0];
        coordinate[i][1] = coordinate[i+1][1];
        coordinate[i][2] = coordinate[i+1][2];
    }
    dis = point_distance(x, y, z, coordinate[8][0], coordinate[8][1], coordinate[8][2]);
    coordinate[9][0] = x;
    coordinate[9][1] = y;
    coordinate[9][2] = z;
    if(dis < 0.25 && dis > 0.001 && judgement()){
        for(c=0; c<=4; c++){
            v[c][0] = (coordinate[c+5][0] - coordinate[c][0]) / time;
            v[c][1] = (coordinate[c+5][1] - coordinate[c][1]) / time;
            v[c][2] = (coordinate[c+5][2] - coordinate[c][2]) / time;
        }
        v[5][0] = (v[0][0] + v[1][0] + v[2][0] + v[3][0] + v[4][0]) / 5;
        v[5][1] = (v[0][1] + v[1][1] + v[2][1] + v[3][1] + v[4][1]) / 5;
        v[5][2] = (v[0][2] + v[1][2] + v[2][2] + v[3][2] + v[4][2]) / 5;
        if(v[5][0]*0.04<0.25 && v[5][1]*0.04<0.25 && v[5][2]*0.04<0.25){
            x = coordinate[9][0] + v[5][0] * 0.04;
            y = coordinate[9][1] + v[5][1] * 0.04;
            z = coordinate[9][2] + v[5][2] * 0.04;
            anti_coordinate_transformation(x, y, z, angle_pitch, angle_yaw);
        } else{
            anti_coordinate_transformation(x, y, z, angle_pitch, angle_yaw);
        }
    } else{
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
    absolute_z = - x * sin(ptz_yaw) - y * sin(ptz_pitch) * cos(ptz_yaw) + z * cos(ptz_pitch) * cos(ptz_yaw);
    x = absolute_x;
    y = absolute_y;
    z = absolute_z;
}

double Predictor::point_distance(double x_1, double y_1, double z_1, double x_2, double y_2, double z_2) {
    double distance;
    distance = sqrt(((x_2 - x_1) * (x_2 - x_1) + (y_2 - y_1) * (y_2 - y_1) + (z_2 - z_1) * (z_2 - z_1)));
    return distance;
}

bool Predictor::judgement() {
    int i;
    double count = 0;
    for(i=0; i<10; i++){
        if(fabs(coordinate[i][0]) > 0.0001 && fabs(coordinate[i][1]) > 0.0001 && fabs(coordinate[i][2]) > 0.001){
            count += 1;
        }
    }
    if(count == 10){
        return true;
    }
}

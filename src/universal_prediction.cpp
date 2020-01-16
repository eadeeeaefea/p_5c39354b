/****************************************************************************
 *
 * Author: Bruce House
 * Date: 2020-01-14
 ***************************************************************************/
#include "universal_prediction.h"
#include "anglesolver.h"

UniversalPredictionKalman::UniversalPredictionKalman() {
}

UniversalPredictionKalman::~UniversalPredictionKalman() {
}

void UniversalPredictionKalman::rune_predict(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {

}

void UniversalPredictionKalman::armor_predict(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {

}

void UniversalPredictionKalman::coordinate_transformation(double &x, double &y, double &z, double ptz_pitch,
                                                          double ptz_yaw) {
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

void UniversalPredictionKalman::anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch,
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


/******************************************************
*******************************************************
*******************************************************
******************************************************/
UniversalPredictionDeeplearning::UniversalPredictionDeeplearning() {
}

UniversalPredictionDeeplearning::~UniversalPredictionDeeplearning() {
}

void UniversalPredictionDeeplearning::rune_predict(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {

}

void UniversalPredictionDeeplearning::armor_predict(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {

}

void UniversalPredictionDeeplearning::coordinate_transformation(double &x, double &y, double &z, double ptz_pitch,
                                                                double ptz_yaw) {
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

void UniversalPredictionDeeplearning::anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch,
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
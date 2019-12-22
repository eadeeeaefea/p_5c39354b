/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Update: Bruce Hou

 Detail:
 *****************************************************************************/

#include "anglesolver.h"

using namespace std;


AngleSolver::AngleSolver() {

}

AngleSolver::~AngleSolver() {

}

double AngleSolver::getOriginPitch() {
    return origin_pitch;
}

double AngleSolver::getOriginYaw() {
    return origin_yaw;
}

void AngleSolver::setOriginPitch(double pitch) {
    origin_pitch = pitch;
}

void AngleSolver::setOriginYaw(double yaw) {
    origin_yaw = yaw;
}


void AngleSolver::init() {
    yaw_offset_ = 0.0;
    pitch_offset_ = 0.0;
}

void AngleSolver::run(double x, double y, double z, double v,
                      double &yaw, double &pitch, double ptz_pitch) {
#ifdef RUNNING_TIME
    Timer timer;
    timer.start();
#endif

    if (parabolaSolve(sqrt(x * x + z * z), y, v, pitch, ptz_pitch)) {
        if (z > 2.5) {
            pitch = -pitch - pitch_offset_;
            yaw = atan(x / z) / PI * 180 - yaw_offset_;
        } else {
            pitch = -pitch - 0.3;
            yaw = atan(x / z) / PI * 180;
        }
    } else {
        pitch = 0.0;
        yaw = 0.0;
    }

#ifdef RUNNING_TIME
    cout << "solve angle time: " << timer.getTime() << "ms" << endl;
    timer.stop();
#endif
}



double AngleSolver::get_yaw_offset() {
    return yaw_offset_;
}

double AngleSolver::get_pitch_offset() {
    return pitch_offset_;
}

void AngleSolver::set_yaw_offset(double yaw_offset) {
    yaw_offset_ = yaw_offset;

}

void AngleSolver::set_pitch_offset(double pitch_offset) {
    pitch_offset_ = pitch_offset;
}

bool AngleSolver::parabolaSolve(double x, double y, double v, double &theta, double ptz_pitch) {
    /**
    double time;
    double res, res_1;
    static const double g = 9.7988;
    time = 2.0 * ((y * g + v * v) - sqrt(pow(g * y + v * v, 2.0)- (x * x + y * y) * g * g)) / (g * g);
    time = sqrt(time);
    res_1 = (y - 0.5 * g * time * time) / v / time;
    res = asin(res_1);
    theta = res * 180 / PI;
    if(isnan(theta) || isinf(theta)){
     theta = 0;
     return false;
    }
    return true;
    //cout << alpha << endl;
    //cout << "(" << x << "," << y << ")" << endl;
    **/

    double time, time_square;
    static const double g = 9.7988;
    double res, res_1;
    double delta_angle;
    double x_bar;
    double y_bar;
    delta_angle = ptz_pitch * PI / 180;
    x_bar = x * cos(delta_angle) - y * sin(delta_angle);
    y_bar = x * sin(delta_angle) + y * cos(delta_angle);
    time_square =
            2.0 * ((y_bar * g + v * v) - sqrt(pow(g * y_bar + v * v, 2.0) - (x_bar * x_bar + y_bar * y_bar) * g * g)) /
            (g * g);
    time = sqrt(time_square);
    res_1 = (y_bar - 0.5 * g * time * time) / v / time;
    res = asin(res_1);
    theta = res * 180 / PI;
    theta = theta - ptz_pitch;
    if (isnan(theta) || isinf(theta)) {
        theta = 0;
        return false;
    }
    return true;

}

double AngleSolver::parabolaDeltaY(double x, double y, double v, double theta) {
    static const double g = 9.7988;

    double vx = v * cos(theta);
    double vy = v * sin(theta);
    double t = x / vx;
    double delta_y = vy * t + 0.5 * g * t * t - y;  // 云台坐标系y轴向下，g向下，故g的符号为正
    // double a = ((-k) / m) * pow(m*g/k, 0.5);
    // double b = atan(vy / pow(m*g/k, 0.5));
    // double t = (m / k / vx) * exp(k*x/m - 1);
    // double delta_y = pow(m*g/k, 0.5) * log(1/cos(a*t+b)) / a - log(1/cos(b)) / a - y;
    return delta_y;
}

/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#include "anglesolver.h"

using namespace std;


AngleSolver::AngleSolver() {

}

AngleSolver::~AngleSolver() {

}

void AngleSolver::init() {
    yaw_offset_ = 0.0;
    pitch_offset_ = 0.0;
}

void AngleSolver::run(double x, double y, double z, double v,
                      double &yaw, double &pitch) {
#ifdef RUNNING_TIME
    Timer timer;
    timer.start();
#endif

    if (parabolaSolve(sqrt(x*x+z*z), y, v, pitch)) {
        pitch = -pitch + pitch_offset_;
        yaw = atan(x / z) / PI * 180 + yaw_offset_;
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

bool AngleSolver::parabolaSolve(double x, double y, double v, double &theta) {
    double min_theta = atan(y / x) - 0.1;
    double max_theta = min_theta + 0.5;

    double flag1 = parabolaDeltaY(x, y, v, min_theta) * parabolaDeltaY(x, y, v, max_theta);
    double flag2 = parabolaDeltaY(x, y, v, min_theta) - parabolaDeltaY(x, y, v, max_theta);
    double mid;

    if (flag1 > 0) {
        return 0;
    } else if (parabolaDeltaY(x, y, v, max_theta) == 0) {
        theta = max_theta / PI * 180;
        return 1;
    } else {
        if (flag2 < 0) {
            while (max_theta - min_theta > 0.0001) {
                mid = (max_theta + min_theta) / 2;
                if (parabolaDeltaY(x, y, v, mid) < 0) {
                    min_theta = mid;
                } else if (parabolaDeltaY(x, y, v, mid) > 0) {
                    max_theta = mid;
                } else {
                    break;
                }
            }
            theta = max_theta / PI * 180;
            return 1;
        } else if (flag2 > 0) {
            while (max_theta - min_theta > 0.0001) {
                mid = (max_theta + min_theta) / 2;
                if (parabolaDeltaY(x, y, v, mid) < 0) {
                    max_theta = mid;
                } else if (parabolaDeltaY(x, y, v, mid) > 0) {
                    min_theta = mid;
                } else {
                    break;
                }
            }
            theta = max_theta / PI * 180;
            return 1;
        } else {
            theta = max_theta / PI * 180;
            return 1;
        }
    }
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

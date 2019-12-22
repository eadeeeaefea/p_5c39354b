/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.11.5

 Detail:
 *****************************************************************************/

#include "predictor.h"
#include <cmath>
#include <iostream>

using std::cout;
using std::endl;


Predictor::Predictor() {

}

Predictor::~Predictor() {

}

void Predictor::init() {
    x_filter.init();
    y_filter.init();
    z_filter.init();
    // first = 1;
    // coeff = 1.5;
    // x_last = 0.0;
    // y_last = 0.0;
    // z_last = 0.0;
}

void Predictor::run(double &x, double &y, double &z) {
    if (fabs(x) < 0.01 && fabs(y) < 0.01 && fabs(z) < 0.01) {
        init();
    } else {
        x = x_filter.run(x);
        y = y_filter.run(y);
        z = z_filter.run(z);
        // if (first) {
        //     x_last = x;
        //     y_last = y;
        //     z_last = z;
        //     first = 0;
        // } else {
        //     x += coeff * (x - x_last);
        //     if (fabs(x - x_last) > 0.5) {
        //         x = x_last;
        //         x_last = 0.0;
        //         first = 1;
        //     } else {
        //         x_last = x;
        //     }
        //
        //     y += coeff * (y - y_last);
        //     if (fabs(y - y_last) > 0.5) {
        //         y = y_last;
        //         y_last = 0.0;
        //         first = 1;
        //     } else {
        //         y_last = y;
        //     }
        //
        //     z += coeff * (z - z_last);
        //     if (fabs(z - z_last) > 0.5) {
        //         z = z_last;
        //         z_last = 0.0;
        //         first = 1;
        //     } else {
        //         z_last = z;
        //     }
        // }
    }
}

KalmanFilter1d::KalmanFilter1d() {
    x.resize(2);
    z.resize(1);
    A.resize(2, 2);
    P.resize(2, 2);
    Q.resize(2, 2);
    R.resize(1, 1);
    H.resize(1, 2);
}

KalmanFilter1d::~KalmanFilter1d() {

}

void KalmanFilter1d::init() {
    delta_t = 0.1;
    s_last = 0.0;
    x.setZero();
    z.setZero();
    A << 1, delta_t, 0, 1;
    P.setIdentity();
    Q.setIdentity() * 200;
    R.setIdentity() * 0.1;
    H << 1, 1;
}

double KalmanFilter1d::run(double s) {
    static double v, s_predict;
    v = (s - s_last) / delta_t;
    if (fabs(v) > 10.0)     v = 0.0;

    s_last = s;
    s_predict = s_last + 0.5 * (s_last - predict(s, v));
    // cout << s_last << "  " << s_predict << "  " << v << "\n";
    return s_predict;
}

double KalmanFilter1d::predict(double s, double v) {
    static Eigen::MatrixXd At;
    x(1) = v;
    x = A * x;
    At = A.transpose();
    P = A * P * At + Q;

    update(s);

    return x(0);
}

void KalmanFilter1d::update(double s) {
    static Eigen::MatrixXd temp1, temp2, Ht, Kk;
    static Eigen::MatrixXd I = Eigen::MatrixXd::Identity(2,2);
    Ht = H.transpose();
    temp1 = H * P * Ht + R;
    temp2 = temp1.inverse();
    Kk = P * Ht * temp2;
    z << s;
    x = x + Kk * (z - H * x);
    P = (I - Kk * H) * P;
}

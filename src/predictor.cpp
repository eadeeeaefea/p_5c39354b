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
    delta_t = 0.12;
    x_last = 0.0;
    y_last = 0.0;
    z_last = 0.0;
}

void Predictor::run(double  x_current, double  y_current, double  z_current,
                    double &x_predict, double &y_predict, double &z_predict) {
    if (fabs(x_current) < 0.0001 &&
        fabs(y_current) < 0.0001 &&
        fabs(z_current) < 0.0001) {

        init();
        x_predict = 0.0;
        y_predict = 0.0;
        z_predict = 0.0;
    } else {

        x_predict = uniformMotionSolve(x_last, x_current);
        y_predict = uniformMotionSolve(y_last, y_current);
        z_predict = uniformMotionSolve(z_last, z_current);
    }
    cout << x_current << "    " << y_current << "    " << z_current << "\n"
         << x_predict << "    " << y_predict << "    " << z_predict << "\n";
}

double Predictor::uniformMotionSolve(double &x0, double x1) {
    double v = (x1 - x0) / delta_t;
    double xt = x1 + v * delta_t;
    x0 = x1;
    return xt;
}

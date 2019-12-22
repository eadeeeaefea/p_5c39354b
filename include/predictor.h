/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.11.5

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_PREDICTOR_H
#define HERORM2020_PREDICTOR_H

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

class KalmanFilter1d {
private:
    Eigen::VectorXd x;
    Eigen::VectorXd z;
    Eigen::MatrixXd A;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd H;
    double delta_t, s_last;

public:
    KalmanFilter1d();
    ~KalmanFilter1d();
    void init();
    double run(double s);

private:
    double predict(double s, double v);
    void update(double z);

};

class Predictor {
private:
    KalmanFilter1d x_filter;
    KalmanFilter1d y_filter;
    KalmanFilter1d z_filter;
    // double x_last, y_last, z_last;
    // double coeff;
    // bool first;

public:
    Predictor();
    ~Predictor();
    void init();
    void run(double &x, double &y, double &z);

};


#endif  // HERORM2020_PREDICTOR_H

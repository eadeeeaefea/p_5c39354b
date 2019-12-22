/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.11.5

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_PREDICTOR_H
#define HERORM2020_PREDICTOR_H


class Predictor {
private:
    double delta_t;
    double x_last, y_last, z_last;

public:
    Predictor();
    ~Predictor();
    void init();
    void run(double  x_current, double  y_current, double  z_current,
             double &x_predict, double &y_predict, double &z_predict);

private:
    double uniformMotionSolve(double &x0, double x1);

};


#endif  // HERORM2020_PREDICTOR_H

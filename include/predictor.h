/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.11.5

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_PREDICTOR_H
#define HERORM2020_PREDICTOR_H

class Predictor {
private:
    double coordinate[10][3];
    double v[6][3];
    double delta_t;
    double x_last, y_last, z_last;

public:
    Predictor();
    ~Predictor();
    void init();
    void run(double &x, double &y, double &z,double ptz_pitch, double ptz_yaw);

private:
    void motion_prediction(double& x, double& y, double& z, double ptz_pitch, double ptz_yaw);
    void coordinate_transformation(double& x, double& y, double& z, double ptz_pitch, double ptz_yaw);
    void anti_coordinate_transformation(double& x, double& y, double& z, double ptz_pitch, double ptz_yaw);
    double point_distance(double x_1, double y_1, double z_1, double x_2, double y_2, double z_2);
    bool judgement();
};


#endif  // HERORM2020_PREDICTOR_H

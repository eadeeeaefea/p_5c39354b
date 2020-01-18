/****************************************************************************
 *
 * Author: Bruce House
 * Date: 2020-01-14
 ***************************************************************************/
#include <Eigen/Dense>
#include <iostream>

using namespace std;

//kalman 每一个函数可以多次调用, 不能使用任何的共享参数,二维预测和一维预测方案未确定
class UniversalPredictionKalman {
private:

public:
    UniversalPredictionKalman();

    ~UniversalPredictionKalman();

    void rune_predict(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    void armor_predict(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

private:
    void coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    void anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);
};

class UniversalPredictionDeeplearning {
private:

public:
    UniversalPredictionDeeplearning();

    ~UniversalPredictionDeeplearning();

    void rune_predict(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    void armor_predict(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

private:
    void coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    void anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);


};
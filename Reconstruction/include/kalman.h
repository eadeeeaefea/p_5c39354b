#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

class KF {
private:
    KalmanFilter kf;
    Mat state = Mat::zeros(2, 1, CV_32F);                                     //state(角度，△角度)
    Mat processNoise = Mat::zeros(2, 1, CV_32F);
    Mat measurement = Mat::zeros(1, 1, CV_32F);                 //定义测量值

    vector<float> target; //目标三维点队列
    vector<float> r_target; // 经过卡尔曼滤波后的目标点队列
    const int queue_size = 3;

public:
    KF();

    ~KF();

    void run(float &x);

private:
    void init_kf();

    void update_queue(float x);

    bool judgement();

    float fobject(float x);
};
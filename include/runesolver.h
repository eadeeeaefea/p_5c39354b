/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_RUNESOLVER_H
#define HERORM2020_RUNESOLVER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.h"
#ifdef RUNNING_TIME
#include "timer.h"
#endif

using namespace std;
using namespace cv;


class RuneSolver {
private:

public:
    RuneSolver();
    ~RuneSolver();
    void init();
    void run(const Mat& src, double &x, double &y, double &z);

private:

};


#endif  // HERORM2020_RUNESOLVER_H

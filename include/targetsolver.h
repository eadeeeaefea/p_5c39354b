/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_TARGETSOLVER_H
#define HERORM2020_TARGETSOLVER_H

#include <opencv2/opencv.hpp>
#include "base.h"
#ifdef RUNNING_TIME
#include "timer.h"
#endif

using namespace std;
using namespace cv;


typedef struct Target_t {
    double x;
    double y;
    double z;
}Target;

class TargetSolver {
private:
    Mat camera_matrix, distortion_coeff;
    int solve_algorithm;
    Mat rotate_mat, trans_mat;

public:
    TargetSolver();
    ~TargetSolver();
    void init(const FileStorage &file_storage);
    void run(const RotatedRect &armor, Target &target);

private:
    void solvePnP4Points(const RotatedRect &rect,
                         const bool is_big_armor,
                         Mat &rotate,
                         Mat &trans);
    void camera2ptzTransform(const Mat &camera_position, Target &ptz_position);

};


#endif  // HERORM2020_TARGETSOLVER_H

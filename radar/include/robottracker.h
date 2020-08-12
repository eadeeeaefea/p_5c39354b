/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Zeng Jing on 2020.04.09.

 Update: Bruce Hou on 2020.04.30

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_ROBOTTRACKER_H
#define HERORM2020_ROBOTTRACKER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>
#include <iostream>
#include <opencv2/tracking.hpp>


#ifdef RUNNING_TIME

#include "timer.h"

#endif

using namespace std;
using namespace cv;


class RobotTracker {
private:
//run
    int enemy_target_num = 0;//目标个数
    const int check_thres = 5;//检测稳定帧数的阈值（待调参）
    const int track_thres = 40;//最大跟踪帧数的阈值（计数中包括检测稳定的几帧，比如设置检测稳定帧数的阈值为5帧，设想最多跟踪的帧数是60帧，则该变量值应设置为5+60=65）（待调参）
public:
    RobotTracker();

    ~RobotTracker();

    void run(const Mat &src, int &enemy_check_cnt,vector<Rect> &enemy_rois, Ptr<MultiTracker> &enemy_tracker);

private:
    void check(int &target_num, int roi_num, Ptr<MultiTracker> &tracker, int &check_cnt);//检查检测是否稳定
    void
    track(const Mat &image, int &check_cnt, vector<Rect> &rois, Ptr<MultiTracker> &multitracker, int &target_num);//进行跟踪
};


#endif  // HERORM2020_ROBOTTRACKER_H

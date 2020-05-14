/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Zeng Jing on 2020.04.08

 Update:

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_ROBOTDETECTOR_H
#define HERORM2020_ROBOTDETECTOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>
#include <iostream>
#include <opencv2/tracking.hpp>
#include "base.h"

#ifdef RUNNING_TIME

#include "timer.h"

#endif

using namespace std;
using namespace cv;


class RobotDetector {
private:
//init
    Ptr<BackgroundSubtractorKNN> robot_background;//knn背景提取类创建
    const int history_num = 10;//用于建模的历史帧数
    const int dist2_thres = 1000;
    //像素和样本之间平方距离的阈值，以确定像素是否接近该样本。
    // 此参数不影响后台更新。适当调高此参数可减少噪点数目（需调参）

//run
    Mat gray_image;

public:
    RobotDetector();

    ~RobotDetector();

    void init(const FileStorage &file_storage);

    void run(const Mat &frame,Mat &fgmask);

};

#endif  // HERORM2020_ROBOTDETECTOR_H

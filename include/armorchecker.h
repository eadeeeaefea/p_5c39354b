/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Zeng Jing on 2020.04.08

 Update: Bruce Hou on 2020.04.30

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_ARMORCHECKER_H
#define HERORM2020_ARMORCHECKER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>
#include <iostream>
#include <opencv2/tracking.hpp>
#include "base.h"
#include "armordetector.h"

#ifdef RUNNING_TIME

#include "timer.h"

#endif

using namespace std;
using namespace cv;


class ArmorChecker {
private:
//init
    ArmorDetector armor_detector;
//preprocessFGmask
    //构造各种尺寸的元素以用于形态学变换
    const int erode_size = 2;//腐蚀用元素尺寸（待调参） 图像尺寸缩放时该参数也会随之变化
    const int dilate_size = 15;//膨胀用元素尺寸（待调参） 图像尺寸缩放时该参数也会随之变化
    Mat erode_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size2f(erode_size, erode_size));//腐蚀用元素
    Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size2f(dilate_size, dilate_size));//膨胀用元素

    double mask_thres;//黑白掩码图像素平均值（准确二值化用）
//findRobot
    vector<vector<Point>> robot_contours;
    Rect robot_roi;
    Mat roi_image;
public:
    ArmorChecker();

    ~ArmorChecker();

    void init(FileStorage file_storage);

    void run(const Mat &src, Mat &fgmask,vector<Rect> &enemy_rois, int enemy_color);

private:
    void preprocessFGmask(Mat &fgmask);//黑白掩码图预处理
    void findRobot(const Mat &fgmask, const Mat &src, vector<Rect> &rois);//查找机器人轮廓并框出
    void checkArmor(const Mat &image, vector<Rect> rois, vector<Rect> &enemy_rois, int enemy_color);
    //根据装甲板检测结果进一步判断目标是否为机器人
};
Rect correctROI(Rect original_roi, const Mat &frame);//校正ROI矩形
#endif  // HERORM2020_ARMORCHECKER_H

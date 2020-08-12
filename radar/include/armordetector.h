/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Update: Wang Xiaoyan on 2019.12.19

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_ARMORDETECTOR_H
#define HERORM2020_ARMORDETECTOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.h"

#ifdef COMPILE_WITH_CUDA
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#endif
#ifdef RUNNING_TIME
#include "timer.h"
#endif

using namespace std;
using namespace cv;


class ArmorDetector {
private:
#ifdef DISTORTION_CORRECT
    Mat camera_matrix;
    Mat distortion_coeff;
#endif  // DISTORTION_CORRECT
#ifndef COMPILE_WITH_CUDA
    Mat kernel;
#else
    cv::Ptr<cv::cuda::Filter> kernel;
#endif  // COMPILE_WITH_CUDA
#ifdef BGR
    int gray_thres, subtract_thres;
#endif  // BGR
#ifdef HSV
    int minH_red,  maxH_red,  minS_red,  maxS_red,  minV_red,  maxV_red;
    int minH_blue, maxH_blue, minS_blue, maxS_blue, minV_blue, maxV_blue;
#endif  // HSV

    int kernel_size;
    double min_aspect_ratio, max_aspect_ratio;
    double min_length_ratio, max_length_ratio;
    double max_lightbar_delta, max_armor_angle, max_armor_lightbar_delta;

    Mat processed_image, roi_image;

#ifdef ROI_ENABLE
    Rect roi_rect;
#endif  // ROI_ENABLE

public:
    ArmorDetector();
    ~ArmorDetector();
    void init(const FileStorage &file_storage);
    void run(const Mat &src,
             const int enemy_color,  // 0-red, 1-blue
             RotatedRect &target_armor);
#ifdef ROI_ENABLE
    Rect get_roi_rect();
    void set_roi_rect(const Rect &rect);
#endif

private:
    void Preprocess(const Mat &src, const int enemy_color, Mat &dst);
    void findTarget(const Mat &dst, RotatedRect &target_armor);
    void findArmors(vector<RotatedRect> &lightbars,
                    vector<RotatedRect> &armors,
                    vector<double> &scores);
    void selectTarget(const vector<RotatedRect> &armors,
                      const vector<double> &scores,
                      RotatedRect &target_armor);
    void adjustRotatedRect(RotatedRect &rect);
    void preventROIExceed(int &x, int &y, int &width, int &height);

};

void drawRotatedRect(Mat &src, RotatedRect &rect);

Rect rectCenterScale(Rect rect, Size size);

#endif  // HERORM2020_ARMORDETECTOR_H

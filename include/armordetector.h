/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

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
#endif
#ifdef RUNNING_TIME
#include "timer.h"
#endif

using namespace std;
using namespace cv;


class ArmorDetector {
private:
    // preprocess
#ifdef DISTORTION_CORRECT
    Mat camera_matrix_;
    Mat distortion_coeff_;
    Mat map1_, map2_;
#ifdef COMPILE_WITH_CUDA
    cv::cuda::GpuMat gpu_map1_, gpu_map2_;
#endif
#endif
#ifndef COMPILE_WITH_CUDA  // cpu only
    Mat kernel_;
#ifdef BGR
    Mat gray_image_, subtract_image_;
    vector<Mat> channels_;
#endif
#ifdef HSV
    Mat hsv_image_;
#endif
#else  // use gpu
    cv::cuda::GpuMat gpu_src_, gpu_dst_, kernel_;
#ifdef BGR
    cv::cuda::GpuMat gray_image_, subtract_image_;
    vector<cv::cuda::GpuMat> channels_;
#endif
#ifdef HSV
    cv::cuda::GpuMat hsv_image_;
#endif
#endif
    Mat roi_image_, original_image_, processed_image_;
    int kernel_size_;
#ifdef BGR
    int gray_thres_, subtract_thres_;
#endif
#ifdef HSV
    int minH_red_,  maxH_red_,  minS_red_,  maxS_red_,  minV_red_,  maxV_red_;
    int minH_blue_, maxH_blue_, minS_blue_, maxS_blue_, minV_blue_, maxV_blue_;
#endif

    // contours
    vector<Vec4i> hierarchy_;
    vector<vector<Point> > contours_;
    RotatedRect temp_rrect_;
    vector<RotatedRect> lightbars_;

    // find armors
    double armor_width_, armor_height_, current_ratio_;
    double length_ratio_;
    double lightbar_angle_delta_, armor_angle_, armor_lightbar_delta_;
    bool temp_result_;
    double temp_score_;
    vector<int> subscript_;
    Point2f left_center_, right_center_;
    Point2f left_vertices_[4], right_vertices_[4];
    vector<Point2f> armor_vertices_;
    double min_ratio_, max_ratio_;
    double min_len_ratio_, max_len_ratio_;
    double max_lightbar_angle_, max_armor_angle_, max_armor_lightbar_delta_;
    vector<RotatedRect> armors_;
    vector<double> scores_;

#ifdef ROI_ENABLE
    Rect roi_rect_;
#endif

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
    void Preprocess(const Mat &src, const int enemy_color, Mat &processed_image);
    void findTarget(const Mat &processed_image, RotatedRect &target_armor);
    void findArmors(vector<RotatedRect> &lightbars,
                    vector<RotatedRect> &armors,
                    vector<double> &scores);
    void selectTarget(const vector<RotatedRect> &armors,
                      const vector<double> &scores,
                      RotatedRect &target_armor);
    void adjustRotatedRect(RotatedRect &rect);
    void preventROIExceed(int &x, int &y, int &width, int &height);

};


#endif  // HERORM2020_ARMORDETECTOR_H

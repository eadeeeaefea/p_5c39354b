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

//#define AVERAGE_INTENSITY
#define AREA_RATIO

#ifdef COMPILE_WITH_CUDA
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif
#ifdef RUNNING_TIME
#include "timer.h"
#endif

#define TRACKBAR
#define SHOW_IMAGE

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
    Mat gray_image_, subtract_image_, kernel_;
    vector<Mat> channels_;
#else  // use gpu
    cv::cuda::GpuMat gpu_src_, gpu_dst_;
    cv::cuda::GpuMat gray_image_, subtract_image_, kernel_;
    vector<cv::cuda::GpuMat> channels_;
#endif
    Mat roi_image_;
    int gray_thres_, subtract_thres_, kernel_size_;
    Mat original_image_, processed_image_;

    // contours
    vector<Vec4i> hierarchy_;
    vector<vector<Point> > contours_;
    RotatedRect temp_rrect_;
    vector<RotatedRect> lightbars_;

    // find armors
    double armor_width_, armor_height_, current_ratio_;
    double length_ratio_;
    double lightbar_angle_delta_, armor_angle_, armor_lightbar_delta_;
    double temp_area_ratio_= 0.0;//尝试通过亮度面积与图片面积比减少误识别的参数
    double temp_average_intensity_=0.0;//尝试通过平均亮度减少误识别的参数
    bool temp_result_;
    double temp_score_;
    vector<int> subscript_;
    Point2f left_center_, right_center_;
    Point2f left_vertices_[4], right_vertices_[4];
    vector<Point2f> armor_vertices_;
    double min_ratio_, max_ratio_;
    double min_len_ratio_, max_len_ratio_;
    double max_lightbar_angle_, max_armor_angle_, max_armor_lightbar_delta_;
    double min_area_ratio_=0,max_area_ratio_=1;//尝试通过亮度面积与图片面积比减少误识别的参数
    double min_area_intensity_=0,max_area_intensity_=1;//尝试通过平均亮度减少误识别的参数
    vector<RotatedRect> armors_;
    vector<double> scores_;

    //calculate armor area ratio
    Rect armor_rect_;

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
    inline void calculateAreaRatio(const Mat &processed_image,
                                   Rect armor_rect_,
                                   double &temp_area_ratio_);
    inline void calculateAverageIntensity(const Mat &processed_image,
                                          Rect armor_rect_,
                                          double &temp_area_ratio_);
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

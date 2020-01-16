/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_RUNESOLVER_H
#define HERORM2020_RUNESOLVER_H

#include <vector>
#include <queue>
#include "base.h"
#include <opencv2/opencv.hpp>
#include <algorithm>

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


typedef enum {
    DIR_DEFAULT = 0,
    DIR_CW,
    DIR_CCW,
    DIR_STATIC
} Direction;
typedef enum {
    MODE_DEFAULT = 0,
    MODE_SMALL,
    MODE_BIG,
    MODE_RANDOM
} RuneMode;

class RuneSolver {
public:
    bool shoot;
    bool isLoseAllTargets;
    bool isCalibrated;
private:
    //常量定义
    const float EXP = 0.5;
    //能量板范围
    static const int MIN_RuneSolver_AREA = 500;
    static const int MAX_RuneSolver_AREA = 1700;
    const float MIN_RuneSolver_RATIO = 1.0f;
    const float MAX_RuneSolver_RATIO = 2.5f;
    //箭头范围
    static const int MIN_ARROW_AREA = 1450;
    static const int MAX_ARROW_AREA = 4000;
    const float MIN_ARROW_RATIO = 1.5f;
    const float MAX_ARROW_RATIO = 2.6f;
    //PNP解算 能量板实际尺寸
    const float RuneSolver_HALF_LENGTH = 135.0f;
    const float RuneSolver_HALF_WIDTH = 65.0f;

    const int QUEUE_SIZE = 100;
    //变量部分
    std::vector<cv::Point3f> object_Points;     //世界坐标系下的坐标//PNP
    std::vector<cv::Point2f> image_Points;      //图像坐标系z下的坐标//PNP
    std::vector<cv::Point3f> runesolver_points; //相机坐标系下的runesolver中心的坐标
    cv::Point3f runecenter; //世界坐标系下的坐标
    cv::Point3f center;
    double delta; //最后两帧的角度差
    cv::Point3f normal_vector; // 法向量
    double offset_delta = 0; //人为的调参值

    cv::Mat rotated_vector;                     //旋转向量
    cv::Mat translation_matrix;                 //平移矩阵
    cv::Mat CAMERA_MATRIX;                      //相机内参矩阵
    cv::Mat DISTCOEFFS;//相机畸变参数
    //平面拟合,https://blog.csdn.net/AlonewaitingNew/article/details/95217730
//    cv::Mat MatrixcoefficientA = cv::Mat(3, 3, CV_32F, Scalar(0)); //矩阵系数A
//    cv::Mat MatrixcoefficientB = cv::Mat(3, 1, CV_32F, Scalar(0)); //矩阵系数B
//    cv::Mat flat_result = cv::Mat(3, 1, CV_32F, Scalar(0)); //平面拟合的结果
    //球面拟合
//    cv::Mat SphericalmatrixA = cv::Mat(3, 3, CV_32F, Scalar(0)); //矩阵系数A
//    cv::Mat SphericalmatrixB = cv::Mat(3, 1, CV_32F, Scalar(0)); //矩阵系数B
//    cv::Mat spherical_result = cv::Mat(3, 1, CV_32F, Scalar(0));
//    double meanx2 = 0, meanx = 0, meanx3 = 0;
//    double meany2 = 0, meany = 0, meany3 = 0;
//    double meanz2 = 0, meanz = 0, meanz3 = 0;
//    double meanxy = 0, meanxz = 0, meanyz = 0;
//    double meanx2y = 0, meanx2z = 0;
//    double meany2x = 0, meany2z = 0;
//    double meanz2x = 0, meanz2y = 0;
    //参数
//    float A; //平面参数
//    float B;
//    float C;
//    float x0; //球心坐标
//    float y0;
//    float z0;
    //预测
    cv::Mat predictmatrixA = cv::Mat(2, 2, CV_32F, Scalar(0));
    cv::Mat predictmatrixB = cv::Mat(2, 1, CV_32F, Scalar(0)); //预测矩阵系数
    cv::Mat result_point = cv::Mat(2, 1, CV_32F, Scalar(0));
#ifndef COMPILE_WITH_CUDA
    cv::Mat src;
    cv::Mat bin;
#else
    cv::Mat src;
    cv::Mat bin;
    cv::cuda::GpuMat src_;
    cv::cuda::GpuMat bin_;
#endif
    RuneMode mode;
    cv::RotatedRect target_RuneSolver;                      //当前帧能量板的最小包围矩形
    cv::RotatedRect target_arrow;                       //当前帧箭头
    bool isFindRuneSolver;                                  //标志位：是否已找到当前帧中的能量板
    bool isFindArrow;                                   //标志位：是否已找到当前帧中的箭头
    bool isFindCenterR;                                 //标志位：是否已找到风车中心R,每次激活能量机关只需要识别一次即可
    std::vector<cv::Point> RuneSolver_centers;              //箭头中心点坐标队列
    std::vector<cv::Point> arrow_centers;               //箭头中心点坐标队列
public:
    RuneSolver();

    ~RuneSolver();

    void init(const FileStorage &file_storage);

    void run(const Mat &image, const int enemy_color, double &x, double &y, double &z, double ptz_pitch,double ptz_yaw);

private:

    void preprocess(const int enemy_color);

    bool findRuneSolver();

    bool findArrow();

    bool match();

    void updateQueue();

    void coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

    void anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);


    void solveRealPostiton(const cv::RotatedRect aim);

    bool fit(); //平面拟合返回的值为z = a * x + b * y + c,球拟合返回值为(x0, y0, z0)

    bool predict(); //预测点的队列

};


#endif  // HERORM2020_RUNESOLVER_H

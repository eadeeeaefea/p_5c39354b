
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
    //拟合圆半径范围
    static const int MIN_RADIUS = 80;
    static const int MAX_RADIUS = 90;
    //能量板范围
    static const int MIN_RuneSolver_AREA = 650;
    static const int MAX_RuneSolver_AREA = 1500;
    const float MIN_RuneSolver_RATIO = 1.0f;
    const float MAX_RuneSolver_RATIO = 2.0f;
    //箭头范围
    static const int MIN_ARROW_AREA = 1450;
    static const int MAX_ARROW_AREA = 3050;
    const float MIN_ARROW_RATIO = 1.5f;
    const float MAX_ARROW_RATIO = 2.6f;
    //PNP解算 能量板实际尺寸
    const float RuneSolver_HALF_LENGTH = 135.0f;
    const float RuneSolver_HALF_WIDTH = 65.0f;

    const int QUEUE_SIZE = 300;
    //变量部分
    std::vector<cv::Point3f> object_Points;     //世界坐标系下的坐标
    std::vector<cv::Point2f> image_Points;      //图像坐标系下的坐标
    cv::Mat rotated_vector;                     //旋转向量
    cv::Mat translation_matrix;                 //平移矩阵
    cv::Mat CAMERA_MATRIX;                      //相机内参矩阵
    cv::Mat DISTCOEFFS;//相机畸变参数
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
    cv::RotatedRect predicted_RuneSolver;                   //预测的能量板的最小包围矩形
    cv::RotatedRect target_arrow;                       //当前帧箭头
    bool isFindRuneSolver;                                  //标志位：是否已找到当前帧中的能量板
    bool isFindArrow;                                   //标志位：是否已找到当前帧中的箭头
    bool isFindCenterR;                                 //标志位：是否已找到风车中心R,每次激活能量机关只需要识别一次即可
    std::vector<cv::Point> RuneSolver_centers;              //箭头中心点坐标队列
    std::vector<cv::Point> arrow_centers;               //箭头中心点坐标队列
    cv::Point ellipse_center;                            //拟合椭圆圆心坐标
    cv::Point current_center;                           //当前帧椭圆圆心坐标
    double ellipse_Xaxis;                               //拟合椭圆X半长轴
    double ellipse_Yaxis;                               //拟合椭圆Y半长轴

    //预测
    std::vector<double> angle_array;
    const float ANGLE_OFFSET = 40.0f;
    Direction direction;
public:
    RuneSolver();

    ~RuneSolver();

    void init(const FileStorage &file_storage);

    void run(const Mat &image, const int enemy_color, double &x, double &y, double &z);

private:
    template<typename T>
    float getDistance(T p1, T p2) {
        return sqrt(pow(p1.x - p2.x, 2.0f) + pow(p1.y - p2.y, 2.0f));
    };

    void preprocess(const int enemy_color);

    bool findRuneSolver();

    bool findArrow();

    bool match();

    void updateQueue();

    void solveCurrentCenter();

    bool findCenterR();

//    bool calculate();
    bool calculate_ellipse();

    float toPolarCoordinates(const cv::Point &temp_point, const cv::Point &origin_point);     //计算极坐标系下的角度
    void judgeDirection();                              //判断方向
    void predicting();                                  //预测位置
    bool solveDirection();

    void solveRealPostiton(const cv::RotatedRect aim);

};


#endif  // HERORM2020_RUNESOLVER_H

//
// Created by luzhan on 19-10-4.
//

#ifndef ENERGY_FINDER_ENERGY_HPP
#define ENERGY_FINDER_ENERGY_HPP
#include <vector>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "timer.h"
#include "base.h"

typedef enum {
    DIR_DEFAULT = 0,
    DIR_CW,
    DIR_CCW,
    DIR_STATIC
}Direction;
typedef enum{
    MODE_DEFAULT = 0,
    MODE_SMALL,
    MODE_BIG,
    MODE_RANDOM
}Mode;

class Energy {
public:
    bool shoot;
    bool isLoseAllTargets;
    bool isCalibrated;
private:
    //常量定义
//    const float PI = 3.14159;
    const float EXP = 0.5;
    //拟合圆半径范围
    static const int MIN_RADIUS = 80;
    static const int MAX_RADIUS = 90;
    //能量板范围
    static const int MIN_ENERGY_AREA = 650;
    static const int MAX_ENERGY_AREA = 1500;
    const float MIN_ENERGY_RATIO = 1.0f;
    const float MAX_ENERGY_RATIO = 2.0f;
    //箭头范围
    static const int MIN_ARROW_AREA = 1450;
    static const int MAX_ARROW_AREA = 3050;
    const float MIN_ARROW_RATIO = 1.5f;
    const float MAX_ARROW_RATIO = 2.6f;
    //PNP解算 能量板实际尺寸
    const float ENERGY_HALF_LENGTH = 135.0f;
    const float ENERGY_HALF_WIDTH = 65.0f;

    const int QUEUE_SIZE = 300;
    //变量部分
    std::vector<cv::Point3f> object_Points;     //世界坐标系下的坐标
    std::vector<cv::Point2f> image_Points;      //图像坐标系下的坐标
    cv::Mat rotated_vector;                     //旋转向量
    cv::Mat translation_matrix;                 //平移矩阵
    cv::Mat CAMERA_MATRIX;                      //相机内参矩阵
    cv::Mat DISTCOEFFS;                         //相机畸变参数

    cv::Mat src;
    cv::Mat bin;
    Mode mode;


    cv::RotatedRect target_energy;                      //当前帧能量板的最小包围矩形
    cv::RotatedRect predicted_energy;                   //预测的能量板的最小包围矩形
    cv::RotatedRect target_arrow;                       //当前帧箭头
    bool isFindEnergy;                                  //标志位：是否已找到当前帧中的能量板
    bool isFindArrow;                                   //标志位：是否已找到当前帧中的箭头
    bool isFindCenterR;                                 //标志位：是否已找到风车中心R,每次激活能量机关只需要识别一次即可
    std::vector<cv::Point> energy_centers;              //箭头中心点坐标队列
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
    Energy();
    ~Energy();
    void run(double &x, double &y, double &z);
    void setSrc(cv::Mat src);
    cv::Mat getSrc();
private:
    template <typename T>
    inline float getDistance(T p1, T p2){
        return sqrt(pow(p1.x - p2.x, 2.0f) + pow(p1.y - p2.y, 2.0f));
    };
    void drawRectangle(cv::Mat &src, cv::RotatedRect rect, cv::Scalar color) {
        cv::Point2f vertices[4];
        rect.points(vertices);
        for (size_t i = 0; i < 4; ++i){
            cv::line(src, vertices[i], vertices[(i+1) % 4], color, 1);
        }
    }
    void preprocess();
    bool findEnergy();
    bool findArrow();
    bool match();
    inline void updateQueue();
    inline void solveCurrentCenter();
    bool findCenterR();
//    bool calculate();
    bool calculate_ellipse();
    inline float toPolarCoordinates(const cv::Point &temp_point, const cv::Point &origin_point);     //计算极坐标系下的角度
    void judgeDirection();                              //判断方向
    void predicting();                                  //预测位置
    bool solveDirection();
    void solveRealPostiton(const cv::RotatedRect aim);
};

#endif //ENERGY_FINDER_ENERGY_HPP

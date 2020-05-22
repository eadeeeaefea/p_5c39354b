/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Bruce Hou
 Update:
 Detail:存放一些不重要的但仍然有价值的函数，有些函数在用，有些没有。该文档提供一些尝试的思路
 *****************************************************************************/
#include <iostream>
#include "opencv2/opencv.hpp"
#include <cmath>

using namespace std;
using namespace cv;


void
coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw); // 将云台坐标系坐标转化为较绝对坐标系下的坐标

void anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch,
                                    double ptz_yaw); // 将较绝对坐标系下的坐标转化为云台坐标系下的坐标

void draw(Mat &src, RotatedRect rect); // 在图像上绘画旋转矩形

Rect get_roi(RotatedRect rect); // 对图像的旋转矩形区域进行处理，得到一个包含旋转矩形的roi
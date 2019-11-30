//
// Created by luzhan on 19-11-12.
//

#include "draw.hpp"
using namespace cv;

void Draw::drawRectangle(cv::Mat &src, cv::RotatedRect rect, cv::Scalar color) {
    Point2f vertices[4];
    int i;
    rect.points(vertices);
    for (i=0; i<4; ++i){
        line(src, vertices[i], vertices[(i+1) % 4], color, 1);
    }
}
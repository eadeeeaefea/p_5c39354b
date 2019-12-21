//
// Created by luzhan on 19-12-21.
//

#ifndef HERORM2020_HERO_COMMON_H
#define HERORM2020_HERO_COMMON_H

#include <opencv2/opencv.hpp>

inline void drawRotatedRect(cv::Mat &src, cv::RotatedRect &rect)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; ++i)
        cv::line(src, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0), 2, 8);
}
#endif //HERORM2020_HERO_COMMON_H

//
// Created by luzhan on 19-11-12.
//

#ifndef RUNE_DRAW_HPP
#define RUNE_DRAW_HPP

#include <opencv2/opencv.hpp>


class Draw {
private:

public:
    void drawRectangle(cv::Mat &src, cv::RotatedRect rect, cv::Scalar color);
};


#endif //RUNE_DRAW_HPP

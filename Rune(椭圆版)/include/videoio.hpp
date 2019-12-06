//
// Created by luzhan on 19-11-3.
//

#ifndef BASE_VIDEOIO_HPP
#define BASE_VIDEOIO_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

class VideoIO {
private:
    cv::VideoCapture cap;
    cv::VideoWriter output;
    const cv::Size DefaultSize = cv::Size(640, 480);
public:
private:
public:
    VideoIO();
    explicit VideoIO(const std::string &input_file_path);
    ~VideoIO();
    bool read(cv::Mat &src);
    bool write(const cv::Mat &src);
};


#endif //BASE_VIDEOIO_HPP

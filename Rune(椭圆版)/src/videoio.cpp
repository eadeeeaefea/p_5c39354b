//
// Created by luzhan on 19-11-3.
//

#include "videoio.hpp"
using namespace cv;
using std::cout;
using std::string;

VideoIO::VideoIO() {
    output.open("效果.avi", CV_FOURCC('M','J','P','G'), 30, DefaultSize);
    if (!output.isOpened()){
        cout<<"视频文件保存失败！\n";
        exit(-1);
    }
}
VideoIO::VideoIO(const std::string &input_file_path) {

    cap.open(input_file_path);
    if (!cap.isOpened()){
        cout<<"视频文件读取失败！\n";
        exit(-1);
    }
    output.open("效果.avi", CV_FOURCC('M','J','P','G'), 30, DefaultSize);
    if (!output.isOpened()){
        cout<<"视频文件保存失败！\n";
        exit(-1);
    }
}

VideoIO::~VideoIO() {
    cap.release();
    output.release();
}

bool VideoIO::read(cv::Mat &src) {
    if (!cap.read(src)){
        cout << "读取源图像失败！\n";
        return false;
    }else {
        resize(src, src, DefaultSize);
        return true;
    }
}

bool VideoIO::write(const cv::Mat &src) {
    output<<src;
}
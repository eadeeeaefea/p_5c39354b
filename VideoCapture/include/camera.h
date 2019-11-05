#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#include <exception>

using  namespace std;
using  namespace cv;

class Camera
{
public:

private:
    VideoCapture cap_;
    string camera_name_;

public:
    Camera(string dev_name,
           int frame_width = 1920,
           int frame_height = 1080,
           int exposure_time = 10);
    Camera();
    ~Camera();

    void open(string dev_name,
              int frame_width = 1920,
              int frame_height = 1080,
              int exposure_time = 10);
    void open();
    bool isOpen();

    void getImage(Mat &image);
    Mat getImage();

private:
        
};

class CameraException : public exception
{
private:
    string error_;

public:
    explicit CameraException(const string &msg) : error_(msg) {}
    virtual const char * what(void) const noexcept override
    {
        return error_.c_str();
    }
};


#endif // CAMERA_H
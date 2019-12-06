/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_WORKSPACE_H
#define HERORM2020_WORKSPACE_H

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <sstream>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include "anglesolver.h"
#include "mvcamera.h"
#include "serialport.h"
#include "timer.hpp"
#include "energy.hpp"
#include "videoio.hpp"
using namespace std;
using namespace cv;

typedef struct Target_t {
    double x;
    double y;
    double z;
}Target;

typedef struct SendPack_t {
    int mode;
    double yaw;
    double pitch;
}SendPack;

typedef struct ReadPack_t {
    int enemy;  // 0-red, 1-blue
    int mode;
    double pitch;
    double yaw;
}ReadPack;

class Workspace {
private:
    enum Mode {
        ARMOR,
        RUNE
    };
    VideoIO video;
    mutex image_buffer_mutex;

    AngleSolver angle_solver;
    MVCamera mv_camera;
    SerialPort serial_port;
    Energy energy;

    vector<Mat> image_buffer_;
    int max_image_buffer_size_;
    Mat current_frame_;
    RotatedRect target_armor_;
    Target target_;
    SendPack send_pack_;
    ReadPack read_pack_;

public:
    Workspace();
    ~Workspace();
    void init();
    void run();

private:
    void imageReceivingFunc();
    void imageProcessingFunc();
    void messageCommunicatingFunc();
    void openSerial();

};


#endif  // HERORM2020_WORKSPACE_H

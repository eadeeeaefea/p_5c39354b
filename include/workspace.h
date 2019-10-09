/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

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
#include <opencv2/opencv.hpp>

#include "base.h"
#include "armordetector.h"
#include "targetsolver.h"
#include "anglesolver.h"
#include "runesolver.h"
#include "mvcamera.h"
#include "serialport.h"
#ifdef RUNNING_TIME
#include "timer.h"
#endif

using namespace std;
using namespace cv;


typedef struct SendPack_t {
    double yaw;
    double pitch;
}SendPack;

typedef struct ReadPack_t {
    int enemy;  // 0-red, 1-blue
    int mode;
}ReadPack;

class Workspace {
private:
    enum Mode {
        ARMOR,
        RUNE
    };

    mutex image_buffer_mutex;

    ArmorDetector armor_detector;
    TargetSolver target_solver;
    AngleSolver angle_solver;
    MVCamera mv_camera;
    SerialPort serial_port;
    RuneSolver rune_solver;

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
    void init(const char *uart_name, const FileStorage &file_storage);
    void run();

private:
    void imageReceivingFunc();
    void imageProcessingFunc();
    void messageCommunicatingFunc();

};


#endif  // HERORM2020_WORKSPACE_H

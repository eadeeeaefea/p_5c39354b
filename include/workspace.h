/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_WORKSPACE_H
#define HERORM2020_WORKSPACE_H

#include <unistd.h>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <sstream>
#include <opencv2/opencv.hpp>

#include "base.h"
#include "mvcamera.h"
#include "serialport.h"
#include "armordetector.h"
#include "targetsolver.h"
#include "anglesolver.h"
#include "runesolver.h"
#include "predictor.h"
#include "cannode.h"

#ifdef RUNNING_TIME
#include "timer.h"
#endif

using namespace std;
using namespace cv;


class Workspace 
{
private:
    mutex image_buffer_mutex;

    ArmorDetector armor_detector;
    TargetSolver target_solver;
    AngleSolver angle_solver;
    MVCamera mv_camera;
    SerialPort serial_port;
    RuneSolver rune_solver;
    Predictor predictor;
    CanNode can_node;

    vector<Mat> image_buffer_;
    int max_image_buffer_size_;
    Mat current_frame_;
    RotatedRect target_armor_;
    Target target_;
    SendPack send_pack_;
    ReadPack read_pack_;

    // plot used
    SerialPort plot_serial;
    PlotPack plot_pack_;

public:
    Workspace();
    ~Workspace();
    void init();
    void run();

private:
    void imageReceivingFunc();
    void imageProcessingFunc();
    void messageCommunicatingFunc();
    void openSerialPort();

};


#endif  // HERORM2020_WORKSPACE_H

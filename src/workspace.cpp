/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#include "workspace.h"


Workspace::Workspace() {

}

Workspace::~Workspace() {

}

void Workspace::init(const char *uart_name, const FileStorage &file_storage) {
    armor_detector.init(file_storage);
    target_solver.init(file_storage);
    angle_solver.init();
    rune_solver.init();
#ifndef TEST
    mv_camera.open(640, 480, 400);
#ifndef CAMERA_ONLY
    serial_port.init(uart_name);
#endif
#endif

    max_image_buffer_size_ = 20;
    send_pack_.yaw = 0.0;
    send_pack_.pitch = 0.0;
    read_pack_.enemy = 0;
    read_pack_.mode = 0;
}

void Workspace::run() {
#ifndef TEST
    thread image_receiving_thread(&Workspace::imageReceivingFunc, this);
#ifndef CAMERA_ONLY
    thread message_communicating_thread(&Workspace::messageCommunicatingFunc, this);
#endif
#endif
#ifndef SAVE_VIDEO_ONLY
    thread image_processing_thread(&Workspace::imageProcessingFunc, this);
#endif

#ifndef TEST
    image_receiving_thread.join();
#ifndef CAMERA_ONLY
    message_communicating_thread.join();
#endif
#endif
#ifndef SAVE_VIDEO_ONLY
    image_processing_thread.join();
#endif
}

void Workspace::imageReceivingFunc() {
#if defined(SAVE_VIDEO) || defined(SAVE_VIDEO_ONLY)
    Mat image;
    VideoWriter writer(VIDEO_SAVED_PATH, VideoWriter::fourcc('M','J','P','G'), 30, Size(640,480));
#endif
    while (1) {
#if defined(SAVE_VIDEO) || defined(SAVE_VIDEO_ONLY)
        mv_camera.getImage(image);
        writer.write(image);
#ifdef SAVE_VIDEO_ONLY
        imshow("current", image);
        waitKey(30);
#endif
#ifdef SAVE_VIDEO
        if (image_buffer_.size() < max_image_buffer_size_) {
            image_buffer_.push_back(image);
        }
#endif
#else
        if (image_buffer_.size() < max_image_buffer_size_) {
            image_buffer_.push_back(mv_camera.getImage());
        }
#endif
    }
}

void Workspace::imageProcessingFunc() {
#ifdef SHOW_IMAGE
    Mat src;
    ostringstream ostr;
#endif
#ifdef TEST
#if TEST == 1
    current_frame_ = imread(IMAGE_PATH);
#elif TEST == 2
    VideoCapture cap(VIDEO_PATH);
#else
    cout << "wrong TEST value, make sure TEST = 1 or 2" << endl;
    exit(0);
#endif
#endif

    while (1) {
#ifndef TEST
        if (!image_buffer_.empty()) {
#else
        if (1) {
#endif
#ifdef RUNNING_TIME
            static Timer mutex_timer;
            mutex_timer.start();
#endif
#ifndef TEST
            image_buffer_mutex.lock();

            current_frame_ = image_buffer_.back();
            image_buffer_.clear();

            image_buffer_mutex.unlock();
#endif
#ifdef RUNNING_TIME
            cout << "lock time: " << mutex_timer.getTime() << "ms" << endl;
            mutex_timer.stop();
#endif
#if TEST == 1

#elif TEST == 2
            cap >> current_frame_;
#endif  // 这里不加#else因上面已经判断过
#ifdef SHOW_IMAGE
            src = current_frame_.clone();
#endif
#ifdef ARMOR_ONLY
            read_pack_.mode = Mode::ARMOR;
#endif
#ifdef RUNE_ONLY
            read_pack_.mode = Mode::RUNE;
#endif
#ifdef ENEMY_COLOR
            read_pack_.enemy = ENEMY_COLOR;
#endif
            if (read_pack_.mode == Mode::ARMOR) {

                armor_detector.run(current_frame_, read_pack_.enemy, target_armor_);
                target_solver.run(target_armor_, target_);
                angle_solver.run(target_.x, target_.y, target_.z, 20, send_pack_.yaw, send_pack_.pitch);

            } else if (read_pack_.mode == Mode::RUNE) {

                rune_solver.run(current_frame_, target_.x, target_.y, target_.z);
                angle_solver.run(target_.x, target_.y, target_.z, 28, send_pack_.yaw, send_pack_.pitch);

            } else {
                continue;
            }
#ifndef TEST
            serial_port.sendData(send_pack_.yaw, send_pack_.pitch);
#endif
            // cout << "x: " << target_.x << "\t"
            //      << "y: " << target_.y << "\t"
            //      << "z: " << target_.z << "\n"
            //      << "yaw: " << send_pack_.yaw << "\t"
            //      << "pitch: " << send_pack_.pitch << endl;
#ifdef TRACKBAR
            namedWindow("current_frame", 1);

            int yaw_offset = static_cast<int>(angle_solver.get_yaw_offset() * 100.0);
            createTrackbar("yaw_offset", "current_frame", &yaw_offset, 500, 0, 0);
            angle_solver.set_yaw_offset(static_cast<double>(yaw_offset) / 100.0);

            int pitch_offset = static_cast<int>(angle_solver.get_pitch_offset() * 100.0);
            createTrackbar("pitch_offset", "current_frame", &pitch_offset, 500, 0, 0);
            angle_solver.set_pitch_offset(static_cast<double>(pitch_offset) / 100.0);
#endif
#ifdef SHOW_IMAGE
            ostr << "yaw: " << send_pack_.yaw;
            putText(src, ostr.str(), Point(20,30), CV_FONT_NORMAL, 1, Scalar(0,255,0));
            ostr.str("");
            ostr << "pitch: " << send_pack_.pitch;
            putText(src, ostr.str(), Point(20,60), CV_FONT_NORMAL, 1, Scalar(0,255,0));
            ostr.str("");
            ostr << "x: " << target_.x;
            putText(src, ostr.str(), Point(20,90), CV_FONT_NORMAL, 1, Scalar(0,255,0));
            ostr.str("");
            ostr << "y: " << target_.y;
            putText(src, ostr.str(), Point(20,120), CV_FONT_NORMAL, 1, Scalar(0,255,0));
            ostr.str("");
            ostr << "z: " << target_.z;
            putText(src, ostr.str(), Point(20,150), CV_FONT_NORMAL, 1, Scalar(0,255,0));
            ostr.str("");

            imshow("current_frame", src);
#ifndef TEST
            waitKey(1);
#endif
#if TEST == 1
            if (waitKey(0) == 27)    exit(0);
#elif TEST == 2
            if (waitKey(30) == 27)    break;
#endif
#endif
        } else {
            continue;
        }
    }
}

void Workspace::messageCommunicatingFunc() {
    while (1) {
        serial_port.readData(read_pack_.enemy, read_pack_.mode);
    }
}

/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Update: Wang Xiaoyan on 2019.10.15

 Detail:
 *****************************************************************************/

#include "workspace.h"


Workspace::Workspace() {

}

Workspace::~Workspace() {

}

void Workspace::init(const FileStorage &file_storage) {
    armor_detector.init(file_storage);
    target_solver.init(file_storage);
    angle_solver.init();
    predictor.init();
//    energy.init();
#ifndef TEST
    mv_camera.open(FRAME_WIDTH, FRAME_HEIGHT, EXPOSURE_TIME);
#ifndef CAMERA_ONLY
    openSerial();
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
    VideoWriter writer(VIDEO_SAVED_PATH, VideoWriter::fourcc('M','J','P','G'), 30, Size(FRAME_WIDTH,FRAME_HEIGHT));
#endif
    while (1) {
        try {
#if defined(SAVE_VIDEO) || defined(SAVE_VIDEO_ONLY)
            mv_camera.getImage(image);
            writer.write(image);
#ifdef SAVE_VIDEO_ONLY
            imshow("current", image);
            if (waitKey(30) == 27)  exit(0);
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
        } catch (MVCameraException &e1) {
            cout << "Camera error." << endl;
            mv_camera.close();
            sleep(1);
            for (int i = 0; i < 10; ++i) {
                try {
                    mv_camera.open(FRAME_WIDTH, FRAME_HEIGHT, EXPOSURE_TIME);
                    if (mv_camera.isOpen())     break;
                } catch (MVCameraException& e2) {
                    cout << "Try to open camera error." << endl;
                    sleep(1);
                }
            }
        }
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
        try {
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
                if (current_frame_.empty())     exit(0);
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
                if (current_frame_.empty())     continue;

                if (read_pack_.mode == Mode::ARMOR) {
                    armor_detector.run(current_frame_, read_pack_.enemy, target_armor_);
                    target_solver.run(target_armor_, target_);
                    predictor.run(target_.x, target_.y, target_.z, target_.x, target_.y, target_.z);
                    angle_solver.run(target_.x, target_.y, target_.z, 20, send_pack_.yaw, send_pack_.pitch, read_pack_.pitch);
                } else if (read_pack_.mode == Mode::RUNE) {

                    energy.setSrc(current_frame_);
                    energy.run(target_.x, target_.y, target_.z);
                    imshow("源图像", energy.getSrc());
                    putText(current_frame_, "read_pitch:" + to_string(read_pack_.pitch), Point(10, 50), 1, 1.6, Scalar(255,255,255));
                    putText(current_frame_, "read_yaw:" + to_string(read_pack_.yaw), Point(10, 100), 1, 1.6, Scalar(255,255,255));

                    if (energy.shoot) {
                        if (!energy.isCalibrated){
                            angle_solver.setOriginPitch(read_pack_.pitch);
                            angle_solver.setOriginYaw(read_pack_.yaw);
                            energy.isCalibrated = true;
                        }
                        cout << target_.x << "  " << target_.y << "  " << target_.z << endl;
                        angle_solver.run(target_.x, target_.y, target_.z, 15, send_pack_.yaw, send_pack_.pitch, read_pack_.pitch);
                        putText(current_frame_, "send_pitch:" + to_string(send_pack_.pitch), Point(10, 150), 1, 1.5, Scalar(255,255,255));
                        putText(current_frame_, "send_yaw:" + to_string(send_pack_.yaw), Point(10, 200), 1, 1.5, Scalar(255,255,255));

                    }else if (energy.isLoseAllTargets && energy.isCalibrated) {
                        cout << target_.x << "  " << target_.y << "  " << target_.z << endl;
                        cout<<"复位!\n";
                        send_pack_.pitch = angle_solver.getOriginPitch() - read_pack_.pitch;
                        send_pack_.yaw = angle_solver.getOriginYaw() - read_pack_.yaw;
                        putText(current_frame_, "O_pitch:" + to_string(angle_solver.getOriginPitch()), Point(10, 250), 1, 1.5, Scalar(255,255,255));
                        putText(current_frame_, "O_yaw:" + to_string(angle_solver.getOriginYaw()), Point(10, 300), 1, 1.5, Scalar(255,255,255));
                        putText(current_frame_, "send_pitch:" + to_string(send_pack_.pitch), Point(10, 150), 1, 1.5, Scalar(255,255,255));
                        putText(current_frame_, "send_yaw:" + to_string(send_pack_.yaw), Point(10, 200), 1, 1.5, Scalar(255,255,255));
//                        waitKey();
                    }
                    imshow("读回的角度", current_frame_);
                    if (waitKey(1) > 0)  {
                        waitKey();
                    }
                } else {
                    continue;
                }
#ifndef CAMERA_ONLY
#ifndef TEST
                serial_port.sendData(0, send_pack_.yaw, send_pack_.pitch);
#endif
#endif
                 cout << "x: " << target_.x << "\t"
                      << "y: " << target_.y << "\t"
                      << "z: " << target_.z << "\n"
                      << "yaw: " << send_pack_.yaw << "\t"
                      << "pitch: " << send_pack_.pitch << endl;
#ifdef TRACKBAR
                namedWindow("current_frame", 1);

                static int yaw_offset = static_cast<int>(angle_solver.get_yaw_offset() * 100.0);
                createTrackbar("yaw_offset", "current_frame", &yaw_offset, 500, 0, 0);
                angle_solver.set_yaw_offset(static_cast<double>(yaw_offset) / 100.0);

                static int pitch_offset = static_cast<int>(angle_solver.get_pitch_offset() * 100.0);
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
                if (waitKey(1) == 27)  exit(0);
#endif
#if TEST == 1
                if (waitKey(0) == 27)     exit(0);
#elif TEST == 2
                if (waitKey(30) == 27)     break;
#endif
#endif
            } else {
                continue;
            }
        } catch (SerialException &e1) {
            cout << "Serial port send error." << endl;
            if (serial_port.isOpen())  serial_port.close();
            sleep(1);
            for (int i = 0; i < 10; ++i) {
                try {
                    openSerial();
                    if (serial_port.isOpen()) break;
                } catch (SerialException &e2) {
                    cout << "Try to open serial port error." << endl;
                    sleep(1);
                }
            }
        } // catch (cv::Exception &e) {  // 预留，出现opencv异常时加入处理
        //
        // }
    }
}

void Workspace::messageCommunicatingFunc() {
    while (1) {
        try {
            serial_port.readData(read_pack_.enemy, read_pack_.mode, read_pack_.pitch, read_pack_.yaw);
        } catch (SerialException &e1) {
            // cout << "Serial port read error." << endl;
            // 因已在imageProcessing线程中作了串口重启，为防止重启冲突造成程序bug，这里只接异常而不处理
            // if (serial_port.isOpen())  serial_port.close();
            // sleep(1);
            // for (int i = 0; i < 10; ++i) {
            //     try {
            //         openSerial();
            //         if (serial_port.isOpen()) break;
            //     } catch (SerialException &e2) {
            //         cout << "Try to open serial port error." << endl;
            //         sleep(1);
            //     }
            // }
        }
    }
}

void Workspace::openSerial() {
    int count = 0;
    string port_name;

    while (count < 3) {
        try {
            port_name = "/dev/ttyUSB" + to_string(count++);
            serial_port.open(port_name);
            if (serial_port.isOpen())  {
                cout << "Open serial successfully in " << port_name << "." << endl;
                return;
            }
        } catch (SerialException &e) {
            cout << "Open " << port_name << " failed." << endl;
        }
    }

    throw SerialException("Open serial failed. Port is not in /dev/ttyUSB0-2");
}

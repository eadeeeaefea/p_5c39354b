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

void Workspace::init() {
    angle_solver.init();
    mv_camera.open();
    openSerial();

    max_image_buffer_size_ = 20;
    send_pack_.yaw = 0.0;
    send_pack_.pitch = 0.0;
    read_pack_.enemy = 0;
    read_pack_.mode = 0;
}

void Workspace::run() {
    thread image_receiving_thread(&Workspace::imageReceivingFunc, this);
    thread message_communicating_thread(&Workspace::messageCommunicatingFunc, this);
    thread image_processing_thread(&Workspace::imageProcessingFunc, this);
    image_receiving_thread.join();
    message_communicating_thread.join();
    image_processing_thread.join();
}

void Workspace::imageReceivingFunc() {

    while (1) {
        if (image_buffer_.size() < max_image_buffer_size_) {
            image_buffer_.push_back(mv_camera.getImage());
        }
    }
}

void Workspace::imageProcessingFunc() {
//    Timer clock;
    int count = 0;
    while (1) {
        try {
            Timer clock;
            if (!image_buffer_.empty()) {

                image_buffer_mutex.lock();

                current_frame_ = image_buffer_.back();
                image_buffer_.clear();

                image_buffer_mutex.unlock();

                if (current_frame_.empty()) continue;
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
                    for (int i=0; i<3; ++i)
                        serial_port.sendData(0, send_pack_.yaw/3, send_pack_.pitch/3);
                    count++;
                }else if (energy.isLoseAllTargets && energy.isCalibrated) {
                    cout << target_.x << "  " << target_.y << "  " << target_.z << endl;
                    cout<<"复位!\n";
                    send_pack_.pitch = angle_solver.getOriginPitch() - read_pack_.pitch;
                    send_pack_.yaw = angle_solver.getOriginYaw() - read_pack_.yaw;
                    putText(current_frame_, "O_pitch:" + to_string(angle_solver.getOriginPitch()), Point(10, 250), 1, 1.5, Scalar(255,255,255));
                    putText(current_frame_, "O_yaw:" + to_string(angle_solver.getOriginYaw()), Point(10, 300), 1, 1.5, Scalar(255,255,255));
                    putText(current_frame_, "send_pitch:" + to_string(send_pack_.pitch), Point(10, 150), 1, 1.5, Scalar(255,255,255));
                    putText(current_frame_, "send_yaw:" + to_string(send_pack_.yaw), Point(10, 200), 1, 1.5, Scalar(255,255,255));
                    for (int i=0; i<3; ++i)
                        serial_port.sendData(0, send_pack_.yaw/3, send_pack_.pitch/3);
                    count++;
                    waitKey();
                }
                imshow("读回的角度", current_frame_);
                video.write(current_frame_);
                clock.stop();
//                if (waitKey(1) == 27){
//                    exit(0);
//                }
                if (waitKey(1) > 0)  {
                    waitKey();
                }



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
            //yaw轴坐标系转换
            if (read_pack_.yaw < 180)
                read_pack_.yaw = -read_pack_.yaw;
            else
                read_pack_.yaw = 360 - read_pack_.yaw;
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

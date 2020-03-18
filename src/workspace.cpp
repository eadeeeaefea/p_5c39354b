/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Update: Wang Xiaoyan on 2019.10.15

 Detail:
 *****************************************************************************/

#include "workspace.h"
#include <cstring>


Workspace::Workspace() {

}

Workspace::~Workspace() {

}

void Workspace::init() {
#if (defined(USE_CAMERA) || defined(USE_SERIAL)) && (defined(TEST) && !defined(USE_CAN))
    cout << "wrong mode, TEST and USE_CAMERA or USE_SERIAL are mutually exclusive." << endl;
    exit(0);
#endif
#if defined(SAVE_VIDEO) && !defined(USE_CAMERA)
    cout << "wrong mode, SAVE_VIDEO need USE_CAMERA." << endl;
    exit(0);
#endif
    // string param_path;
    // param_path = PARAM_PATH + to_string(USE_CAMERA) + ".xml";
    FileStorage file_storage(PARAM_PATH, FileStorage::READ);

    armor_detector.init(file_storage);
    target_solver.init(file_storage);
    angle_solver.init();
//    predictor.init();
    rune_solver.init(file_storage);
#ifdef USE_CAMERA
    mv_camera.open(FRAME_WIDTH, FRAME_HEIGHT, EXPOSURE_TIME);
#endif
#if defined(USE_SERIAL) || defined(PLOT_DATA)
    openSerialPort();
#endif

#ifdef USE_CAN
    can_node.init();
#endif

#ifdef PLOT_DATA
    //TODO: serial exception : repeatly check serial name
    plot_pack.plot_type = 3;
    plot_pack.curve_num = 3;
#endif

    max_image_buffer_size = 10;
    read_pack.enemy_color = 0;
    read_pack.mode = 0;
    send_pack.yaw = 0.0;
    send_pack.pitch = 0.0;
}

void Workspace::run() {
#ifdef USE_CAMERA
    thread image_receiving_thread(&Workspace::imageReceivingFunc, this);
#endif
#if SAVE_VIDEO != 2
    thread image_processing_thread(&Workspace::imageProcessingFunc, this);
#endif
#if defined(USE_SERIAL) || defined(USE_CAN)
    thread message_communicating_thread(&Workspace::messageCommunicatingFunc, this);
#endif

#ifdef USE_CAMERA
    image_receiving_thread.join();
#endif
#if SAVE_VIDEO != 2
    image_processing_thread.join();
#endif
#if defined(USE_SERIAL) || defined(USE_CAN)
    message_communicating_thread.join();
#endif
}

void Workspace::imageReceivingFunc() {
#ifdef SAVE_VIDEO
    Mat image;
    VideoWriter writer(VIDEO_SAVED_PATH, VideoWriter::fourcc('M','J','P','G'), 30, Size(FRAME_WIDTH,FRAME_HEIGHT));
#endif
    while (1) {
        try {
#ifdef RUNNING_TIME
             static Timer timer;
             timer.start();
#endif
#ifdef SAVE_VIDEO
            mv_camera.getImage(image);
            writer.write(image);
#if SAVE_VIDEO == 2
            imshow("current", image);
            if (waitKey(30) == 27)  exit(0);
#elif SAVE_VIDEO == 1
            if (image_buffer.size() < max_image_buffer_size) {
                image_buffer.push_back(image);
            }
#else
            cout << "wrong SAVE_VIDEO value, make sure SAVE_VIDEO = 1 or 2." << endll;
            exit(0);
#endif
#endif  // SAVE_VIDEO
#if defined(USE_CAMERA) && !defined(SAVE_VIDEO)
            if (image_buffer.size() < max_image_buffer_size) {
                image_buffer.push_back(mv_camera.getImage());
            }
#endif
#ifdef RUNNING_TIME
             cout << "get image: " << timer.getTime() << "ms" << endl;
             timer.stop();
#endif
        } catch (MVCameraException &e1) {
            cout << "Camera error." << endl;
            mv_camera.close();
            sleep(1);
            for (int i = 0; i < 5; ++i) {
                try {
                    mv_camera.open(FRAME_WIDTH, FRAME_HEIGHT, EXPOSURE_TIME);
                    if (mv_camera.isOpen()) break;
                } catch (MVCameraException &e2) {
                    cout << "Try to open camera error." << endl;
                    sleep(1);
                }
            }
            if (!mv_camera.isOpen()) exit(1);
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
    current_frame = imread(IMAGE_PATH);
#elif TEST == 2
    VideoCapture cap(VIDEO_PATH);
#else
    cout << "wrong TEST value, make sure TEST = 1 or 2" << endl;
    exit(0);
#endif
#endif  // TEST

    while (1) {
        try {
#ifdef RUNNING_TIME
            static Timer total_timer;
            total_timer.start();
#endif
#ifdef USE_CAMERA
            if (!image_buffer.empty()) {
#else
            if (1) {
#endif  // USE_CAMERA
#ifdef USE_CAMERA
#ifdef RUNNING_TIME
                // static Timer mutex_timer;
                // mutex_timer.start();
#endif  // RUNNING_TIME
                image_buffer_mutex.lock();

                current_frame = image_buffer.back();
                image_buffer.clear();

                image_buffer_mutex.unlock();
#ifdef RUNNING_TIME
                // cout << "lock time: " << mutex_timer.getTime() << "ms" << endl;
                // mutex_timer.stop();
#endif  // RUNNING_TIME
#endif  // USE_CAMERA
#if TEST == 2
                cap >> current_frame;
                if (current_frame.empty()) exit(0);
#endif
#ifdef SHOW_IMAGE
                src = current_frame.clone();
#endif
#ifdef ARMOR_ONLY
                read_pack.mode = Mode::ARMOR;
#endif
#ifdef RUNE_ONLY
                read_pack.mode = Mode::RUNE;
#endif
#ifdef ENEMY_COLOR
                read_pack.enemy_color = ENEMY_COLOR;
#endif
                if (current_frame.empty()) continue;

                if (read_pack.mode == Mode::ARMOR) {
                    armor_detector.run(current_frame, read_pack.enemy_color, target_armor);
                    target_solver.run(target_armor, target);
                    predict.run(target.x, target.y, target.z, 20, send_pack.pitch, send_pack.yaw, 0, 0);
//                    angle_solver.run(target.x, target.y, target.z, 20, send_pack.yaw,
//                                     send_pack.pitch, read_pack.pitch);
//                    cout << "pitch: " << read_pack.pitch << endl;
                    send_pack.mode = 0;

                } else if (read_pack.mode == Mode::RUNE) {
                    read_pack.pitch = 0;
                    read_pack.yaw = 0;
                    if(rune_solver.run(current_frame, read_pack.enemy_color, target.x, target.y, target.z)){
                        rune_solver.predict(target.x, target.y,target.z, 20, send_pack.pitch, send_pack.yaw, read_pack.pitch, read_pack.yaw);
                    }
                } else {
                    continue;
                }
#if defined(USE_SERIAL) && !defined(TEST)
                serial_port.sendData(send_pack);
#endif
#if defined(USE_CAN) && !defined(TEST)
//                send_pack.yaw = 1.5;
//                send_pack.pitch = 1.3;
                can_node.send(send_pack);
//                cout << "sucessfully" << endl;
#endif
#ifdef PLOT_DATA
                plot_pack.plot_value[0] = target.x;
                plot_pack.plot_value[1] = target.y;
                plot_pack.plot_value[2] = target.z;
                plot_serial.sendPlot(plot_pack);
#endif
#ifdef ARMOR_ONLY
//                cout << "x: " << target.x << "\t"
//                     << "y: " << target.y << "\t"
//                     << "z: " << target.z << "\n"
//                     << "yaw: " << send_pack.yaw << "\t"
//                     << "pitch: " << send_pack.pitch << endl;
#endif
//#ifdef RUNE_ONLY
//                cout << "yaw: " << send_pack.yaw << "\t"
//                     << "pitch: " << send_pack.pitch << endl;
//#endif
#ifdef TRACKBAR
                namedWindow("current_frame", 1);

                static int yaw_offset = static_cast<int>(angle_solver.get_yaw_offset() * 100.0) + 500;
                createTrackbar("yaw_offset", "current_frame", &yaw_offset, 1000, 0, 0);
                angle_solver.set_yaw_offset(static_cast<double>(yaw_offset - 500) / 100.0);

                static int pitch_offset = static_cast<int>(angle_solver.get_pitch_offset() * 100.0) + 500;
                createTrackbar("pitch_offset", "current_frame", &pitch_offset, 1000, 0, 0);
                angle_solver.set_pitch_offset(static_cast<double>(pitch_offset - 500) / 100.0);

#endif  // TRACKBAR
#ifdef SHOW_IMAGE
                ostr << "yaw: " << send_pack.yaw;
                putText(src, ostr.str(), Point(20, 30), QT_FONT_NORMAL, 1, Scalar(0, 255, 0));
                ostr.str("");
                ostr << "pitch: " << send_pack.pitch;
                putText(src, ostr.str(), Point(20, 60), QT_FONT_NORMAL, 1, Scalar(0, 255, 0));
                ostr.str("");
                ostr << "x: " << target.x;
                putText(src, ostr.str(), Point(20, 90), QT_FONT_NORMAL, 1, Scalar(0, 255, 0));
                ostr.str("");
                ostr << "y: " << target.y;
                putText(src, ostr.str(), Point(20, 120), QT_FONT_NORMAL, 1, Scalar(0, 255, 0));
                ostr.str("");
                ostr << "z: " << target.z;
                putText(src, ostr.str(), Point(20, 150), QT_FONT_NORMAL, 1, Scalar(0, 255, 0));
                ostr.str("");

                drawRotatedRect(src, target_armor);
                imshow("current_frame", src);
#ifdef USE_CAMERA
                if (waitKey(1) == 27) exit(0);
#endif
#if TEST == 1
                if (waitKey(0) == 27)     exit(0);
#elif TEST == 2
                if (waitKey(30) == 27) break;
#endif
#endif  // SHOW_IMAGE
#ifdef RUNNING_TIME
                cout << "total time: " << total_timer.getTime() << "ms" << endl;
                total_timer.stop();
#endif
            } else {
                continue;
            }
        } catch (SerialException &e1) {
            cout << "Serial port send error." << endl;
            if (serial_port.isOpen()) serial_port.close();
            sleep(1);
            for (int i = 0; i < 5; ++i) {
                try {
                    openSerialPort();
                    if (serial_port.isOpen()) break;
                } catch (SerialException &e2) {
                    cout << "Try to open serial port error." << endl;
                    sleep(1);
                }
            }
            if (!serial_port.isOpen()) exit(1);
        }
    }
}

void Workspace::messageCommunicatingFunc() {
#ifdef USE_SERIAL
    while (1) {
        try {
            ///define ARMOE 和 define RUNEONLY 会被取消因为串口中有读mode的函数
#ifdef ARMOR_ONLY
            serial_port.readData(read_pack);
#endif
#ifdef RUNE_ONLY
            serial_port.readData(read_pack);
        //yaw轴坐标系转换
        if (read_pack.yaw < 180)
            read_pack.yaw = -read_pack.yaw;
        else
            read_pack.yaw = 360 - read_pack.yaw;
#endif
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
#endif
#ifdef USE_CAN
#ifdef ARMOR_ONLY
    while (true) {
        can_node.receive(read_pack);
    }
#endif
#ifdef RUNE_ONLY
    while (true) {
        can_node.receive(read_pack);
        if(read_pack.yaw > 180)
            read_pack.yaw = read_pack.yaw - 360;
    }
#endif
#endif

}

void Workspace::openSerialPort() {
#ifdef PLOT_DATA
    try {
#ifdef USE_SERIAL
        string port_name = "/dev/ttyUSB0";
        serial_port.open(port_name);
        if (serial_port.isOpen()) {
            cout << "Open seiral " + port_name + " successfully.\n";
        }
#endif
        string plot_port = "/dev/ttyUSB1";
        plot_serial.open(plot_port);
        if (plot_serial.isOpen()) {
            cout << "Open plot_serial " + plot_port + " successfully.\n";
            return;
        }
    }
    catch (const SerialException &e) {
        std::cerr << e.what() << '\n';
    }
#else
#ifdef USE_SERIAL
    int count = 0;
    string port_name;

    while (count < 3) {
        try {
            port_name = "/dev/ttyUSB" + to_string(count++);
            serial_port.open(port_name);
            if (serial_port.isOpen()) {
                cout << "Open serial successfully in " << port_name << "." << endl;
                return;
            }
        } catch (SerialException &e) {
            cout << "Open " << port_name << " failed." << endl;
        }
    }

    throw SerialException("Open serial failed. Port is not in /dev/ttyUSB0-2");
#endif //USE_SERIAL
#endif //PLOT_DATA
}

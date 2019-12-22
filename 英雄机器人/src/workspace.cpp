/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Update: Li Haotian on 2019.12

 Detail:
 *****************************************************************************/

#include "workspace.h"


Workspace::Workspace(){}

Workspace::~Workspace(){}

void Workspace::init(){
#if (defined(USE_CAMERA) || defined(USE_SERIAL)) && (defined(TEST))
    cout << "wrong mode, TEST and USE_CAMERA or USE_SERIAL are mutually exclusive." << endl;
    exit(0);
#endif  // (defined(USE_CAMERA) || defined(USE_SERIAL)) && (defined(TEST))

#if defined(SAVE_VIDEO) && !defined(USE_CAMERA)
    cout << "wrong mode, SAVE_VIDEO need USE_CAMERA." << endl;
    exit(0);
#endif  //defined(SAVE_VIDEO) && !defined(USE_CAMERA)

#ifdef USE_CAMERA
    mv_camera.open(FRAME_WIDTH, FRAME_HEIGHT, EXPOSURE_TIME);
#endif  //USE_CAMERA

#if defined(USE_SERIAL) || defined(PLOT_DATA)
    openSerialPort();
#endif  //defined(USE_SERIAL) || defined(PLOT_DATA)

#ifdef PLOT_DATA
    //TODO: serial exception : repeatly check serial name
    plot_pack_.plot_type = 2;
    plot_pack_.curve_num = 1;
#endif  //PLOT_DATA

    FileStorage file_storage(PARAM_PATH, FileStorage::READ);
    armor_detector.init(file_storage);
    target_solver.init(file_storage);
    angle_solver.init();
    predictor.init();
    max_image_buffer_size_ = 10;
    read_pack_.enemy_color = 0;
    read_pack_.mode = 0;
}

void Workspace::run(){
#ifdef USE_CAMERA
    thread image_receiving_thread(&Workspace::imageReceivingFunc, this);
#endif  //USE_CAMERA

    thread image_processing_thread(&Workspace::imageProcessingFunc, this);

#ifdef USE_SERIAL
    thread message_communicating_thread(&Workspace::messageCommunicatingFunc, this);
#endif  //USE_SERIAL

#ifdef USE_CAMERA
    image_receiving_thread.join();
#endif  //USE_CAMERA

    image_processing_thread.join();

#ifdef USE_SERIAL
    message_communicating_thread.join();
#endif  //USE_SERIAL
}

//图像接收线程
void Workspace::imageReceivingFunc(){
#ifdef SAVE_VIDEO
    Mat image;
    VideoWriter writer(VIDEO_SAVED_PATH, VideoWriter::fourcc('M','J','P','G'), 30, Size(FRAME_WIDTH,FRAME_HEIGHT));
#endif  // SAVE_VIDEO

    while (1){
        try{
#ifdef SAVE_VIDEO
            mv_camera.getImage(image);
            writer.write(image);
#if SAVE_VIDEO == 2
            imshow("current", image);
            if (waitKey(30) == 27)  exit(0);
#elif SAVE_VIDEO == 1
            if (image_buffer_.size() < max_image_buffer_size_){
                image_buffer_.emplace_back(image);
            }
#else
            cout << "wrong SAVE_VIDEO value, make sure SAVE_VIDEO = 1 or 2." << endll;
            exit(0);
#endif  //SAVE_VIDEO == 2
#endif  // SAVE_VIDEO

#ifdef USE_CAMERA
            //将图像放进缓冲区
            if (image_buffer_.size() < max_image_buffer_size_){
                image_buffer_.emplace_back(mv_camera.getImage());
            }
#endif  // USE_CAMERA
        }catch (MVCameraException &e1)
        {
            // cout << "Camera error." << endl;
            mv_camera.close();
            sleep(1);
            for (int i = 0; i < 10; ++i){
                try{
                    mv_camera.open(FRAME_WIDTH, FRAME_HEIGHT, EXPOSURE_TIME);
                    if (mv_camera.isOpen())
                        break;
                }catch (MVCameraException& e2)
                {
                    cout << "Try to open camera error." << endl;
                    sleep(1);
                }
            }
            if (!mv_camera.isOpen())
                exit(1);
        }
    }
}

//图像处理线程
void Workspace::imageProcessingFunc(){
#ifdef TEST
#if TEST == 1
    current_frame_ = imread(IMAGE_PATH);
#elif TEST == 2
    VideoCapture cap(VIDEO_PATH);
#else
    cout << "wrong TEST value, make sure TEST = 1 or 2" << endl;
    exit(0);
#endif
#endif  // TEST

    while (1){

#ifdef RUNNING_TIME
        Timer clock;
#endif

        try{
#ifdef USE_CAMERA
            if (!image_buffer_.empty()){
                image_buffer_mutex.lock();
                current_frame_ = image_buffer_.back();
                image_buffer_.clear();
                image_buffer_mutex.unlock();
            }
#ifdef RUNNING_TIME
            clock.stop("线程加锁");
            clock.restart();
#endif  //RUNNING_TIME
#endif  // USE_CAMERA

#if TEST == 2
            cap >> current_frame_;
            if (current_frame_.empty()){
                cout<<"视频读取失败\n";
                exit(0);
            }
#endif  // TEST == 2

#ifdef ARMOR_ONLY
            read_pack_.mode = Mode::ARMOR;
#endif

#ifdef ENEMY_COLOR
            read_pack_.enemy_color = ENEMY_COLOR;
#endif

            if (current_frame_.empty())
                continue;

            if (read_pack_.mode == Mode::ARMOR){
                armor_detector.run(current_frame_, read_pack_.enemy_color, target_armor_);
                target_solver.run(target_armor_, target_);
                // predictor.run(target_.x, target_.y, target_.z, target_.x, target_.y, target_.z);
                angle_solver.run(target_.x, target_.y, target_.z, 20, send_pack_.yaw, send_pack_.pitch, read_pack_.pitch);
                send_pack_.mode = 0;
            }else
            {
                continue;
            }
#ifdef USE_SERIAL
            serial_port.sendData(send_pack_);
#endif
            target_.x = 10.0;
            target_.y = 20.0;
            target_.z = 30.0;
#ifdef PLOT_DATA
            plot_pack_.plot_value[0] = target_.x;
            plot_pack_.plot_value[1] = target_.y;
            plot_pack_.plot_value[2] = target_.z;
            plot_serial.sendPlot(plot_pack_);
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
            Mat src;
            ostringstream ostr;
            src = current_frame_.clone();

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

            drawRotatedRect(src, target_armor_);
            imshow("current_frame", src);
#endif // SHOW_IMAGE

#ifdef USE_CAMERA
            if (waitKey(1) == 27)  exit(0);
#endif  // USE_CAMERA

#ifdef TEST
#if TEST == 1
            if (waitKey(0) == 27)
                exit(0);
#else
            if (waitKey(30) == 27)
                break;
#endif  // TEST == 1
#endif  // TEST

        }catch (SerialException &e1)
        {
            cout << "Serial port send error." << endl;
            if (serial_port.isOpen())  serial_port.close();
            sleep(1);
            for (int i = 0; i < 10; ++i){
                try{
                    openSerialPort();
                    if (serial_port.isOpen())
                        break;
                }catch (SerialException &e2)
                {
                    cout << "Try to open serial port error." << endl;
                    sleep(1);
                }
            }
            if (!serial_port.isOpen())
                exit(1);
        }
#ifdef RUNNING_TIME
        clock.stop("图像处理");
#endif
    }
}

/***************************************************************
  *  @brief     串口接受接收线程,接收STM32发来的数据包
  *  @param     无
  *  @note      无
  *  @Sample usage:     thread message_communicating_thread(&Workspace::messageCommunicatingFunc, this);
 **************************************************************/
void Workspace::messageCommunicatingFunc(){
#ifdef USE_SERIAL
    while (1){
        try{
             serial_port.readData(read_pack_);
        }catch (SerialException &e1)
        {
            // cout << "Serial port read error." << endl;
            // 因已在imageProcessing线程中作了串口重启，为防止重启冲突造成程序bug，这里只接异常而不处理
            // if (serial_port.isOpen())  serial_port.close();
            // sleep(1);
            // for (int i = 0; i < 10; ++i) {
            //     try {
            //         openSerialPort();
            //         if (serial_port.isOpen()) break;
            //     } catch (SerialException &e2) {
            //         cout << "Try to open serial port error." << endl;
            //         sleep(1);
            //     }
            // }
            // if (!serial_port.isOpen())    exit(1);
        }
    }
#endif
}

//上位机绘图会占一个接口,写法有所差异
void Workspace::openSerialPort(){

#ifdef PLOT_DATA
    try
    {
#ifdef USE_SERIAL
        string port_name = "/dev/ttyUSB0";
        serial_port.open(port_name);
        if(serial_port.isOpen())
        {
            cout << "Open seiral " + port_name + " successfully.\n";  
        }
#endif // USE_SERIAL
        string plot_port = "/dev/ttyUSB2";
        plot_serial.open(plot_port);
        if(plot_serial.isOpen())
        {
            cout << "Open plot_serial " + plot_port + " successfully.\n";
            return;
        }
    }
    catch(const SerialException &e)
    {
        std::cerr << e.what() << '\n';
    }
#else
#ifdef USE_SERIAL
    string port_name;
    int count = 0;
    while (count < 3){
        try{
            port_name = "/dev/ttyUSB" + to_string(count++);
            serial_port.open(port_name);
            if (serial_port.isOpen())
            {
                cout << "Open serial successfully in " << port_name << "." << endl;
                return;
            }
        }catch (SerialException &e)
        {
            cout << "Open " << port_name << " failed." << endl;
        }
    }
    throw SerialException("Open serial failed. Port is not in /dev/ttyUSB0-2");
#endif // USE_SERIAL
#endif // PLOT_DATA
}

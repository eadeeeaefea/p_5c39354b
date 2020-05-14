/**
 * @file workspace.cpp
 * @brief 
 * @details 
 * @author Li Haotian on 2020-05-09
 * @email lcyxlihaotian@126.com
 * @update 
 * @version 
 * @license 2020 HITWH HERO-Robomaster Group
 */

#include "workspace.h"

Workspace::Workspace()
{
}

Workspace::~Workspace()
{
}

void Workspace::init()
{
#if (defined(USE_CAMERA) || defined(USE_SERIAL)) && defined(TEST)
    cout << "wrong mode, TEST and (USE_CAMERA or USE_SERIAL) are mutually exclusive." << endl;
    exit(0);
#endif
#if defined(SAVE_VIDEO) && !defined(USE_CAMERA)
    cout << "wrong mode, SAVE_VIDEO need USE_CAMERA." << endl;
    exit(0);
#endif
    FileStorage fs(PARAM_PATH, FileStorage::READ);
    robot_detector.init(fs);
    enemy_tracker = cv::MultiTracker::create();
    armor_checker.init(fs);

//TODO: add specific configuration
#ifdef USE_CAMERA // open camera used
#ifdef USE_ROBOT
    dh_camera.open();
#endif // USE_ROBOT
#ifdef USE_MISSILE

#endif // USE_MISSILE
#endif // USE_CAMERA

#ifdef PLOT_DATA
    plot_pack_.plot_type = 3;
    plot_pack_.curve_num = 3;
#endif
    //TODO: redesign protocol
    read_pack_.enemy_color = ENEMY_COLOR;
    read_pack_.mode = 0;
}

void Workspace::run()
{
//TODO: add TEST macro def for missile proc

//TODO: check whether seperating robot & missile is needed
#ifdef USE_CAMERA
    thread img_recv_thread(&Workspace::imgRecFunc, this);
#endif
//TODO: do not join missile thread immediately, move as a member var
#if SAVE_VIDEO != 2
    thread rob_img_proc_thread(&Workspace::robImgProcFunc, this);
#endif

#ifdef USE_SERIAL
    thread msg_comm_thread(&Workspace::msgCommFunc, this);
#endif

//TODO: check whether seperating robot & missile is needed
#ifdef USE_CAMERA
    img_recv_thread.join();
#endif // USE_CAMERA

#if SAVE_VIDEO != 2
    rob_img_proc_thread.join();
#endif

#if defined(USE_SERIAL)
    msg_comm_thread.join();
#endif
}

//TODO add missile
void Workspace::imgRecvFunc()
{
#ifdef SAVE_VIDEO
    Mat image;
    VideoWriter writer(ROBOT_VIDEO_SAVE_PATH, VideoWriter::fourcc('M', 'J', 'P', 'G'),
                       30, Size(ROBOT_FRAME_WIDTH, ROBOT_FRAME_HEIGHT));
#endif

    while (1)
    {
        try
        {
#ifdef SAVE_VIDEO
            dh_camera.getImage(image);
            writer.write(image);
#if SAVE_VIDEO == 2
            imshow("current", image);
            if (waitKey(30) == 27)
                exit(0);
#elif SAVE_VIDEO == 1
            if (robot_image_buffer_.size() < max_image_buffer_size_)
            {
                robot_image_buffer_.push_back(image);
            }
#else
            cout << "wrong SAVE_VIDEO value, make sure SAVE_VIDEO = 1 or 2." << endll;
            exit(0);
#endif
#endif // SAVE_VIDEO

#if defined(USE_CAMERA) && !defined(SAVE_VIDEO)
            if (robot_image_buffer_.size() < max_image_buffer_size_)
            {
                robot_image_buffer_.push_back(dh_camera.getImage());
            }
#endif
        }
        catch (DHCameraException &e1)
        {
            // cout << "Camera error." << endl;
            dh_camera.close();
            sleep(1);
            for (int i = 0; i < 5; ++i)
            {
                try
                {
                    dh_camera.open(ROBOT_FRAME_WIDTH, ROBOT_FRAME_HEIGHT, ROBOT_EXPOSURE_TIME);
                    if (dh_camera.isOpen())
                        break;
                }
                catch (DHCameraException &e2)
                {
                    cout << "Try to open camera error." << endl;
                    sleep(1);
                }
            }
            if (!dh_camera.isOpen())
                exit(1);
        }
    }
}

void Workspace::robImgProcFunc()
{
    Timer timer_frame;
    int frame_count = 0; //视频帧数计数变量（配合knn背景建模算法使用）

//TODO add missile configuration
#ifdef TEST
#if TEST == 1
    current_frame_ = imread(IMAGE_PATH);
#elif TEST == 2
    VideoCapture cap(ROBOT_VIDEO_READ_PATH);
#else
    cout << "wrong TEST value, make sure TEST = 1 or 2" << endl;
    exit(0);
#endif
#endif // TEST

    while (true)
    {
        try
        {
#ifdef USE_CAMERA
            if (robot_image_buffer_.empty())
                continue;
            else
            {
#else
            if (true)
            {
#endif // USE_CAMERA
#ifdef RUNNING_TIME
                timer_frame.start();
#endif

#ifdef USE_CAMERA
#ifdef RUNNING_TIME
                static Timer mutex_timer;
                mutex_timer.start();
                cout << "before cap >>: " << total_timer.getTime() << "ms\n";
#endif
                image_buffer_mutex.lock();

                current_frame_ = color_image_buffer_.back();
                color_image_buffer_.clear();

                image_buffer_mutex.unlock();

#ifdef RUNNING_TIME
                cout << "lock time: " << mutex_timer.getTime() << "ms" << endl;
                mutex_timer.stop();
#endif
                if (current_frame_.empty())
                    continue;
#endif // USE_CAMERA \
                //TODO: whether re-extract frontground is needed after some frames
#if TEST == 2
                cap >> current_frame_;
                if (current_frame_.empty())
                {
                    cout << "frame empty" << endl;
                    exit(0);
                }
#endif
                frame_count++;

                resize(current_frame_, current_frame_,
                       Size(current_frame_.size().width / image_ratio_,
                            current_frame_.size().height / image_ratio_));
                //为方便查看缩放图片（此操作可能导致腐蚀、膨胀操作中所用元素尺寸需要调整）

                vector<Rect> enemy_rois; // 存放敌对机器人的装甲板

                Mat robot_fgmask;
                robot_detector.run(current_frame_, robot_fgmask); //前景提取查找机器人
                if (frame_count > history_num_)
                { //取完背景提取所需帧数（前景提取稳定）后进行后续处理
                    Mat frame_show = current_frame_.clone();
                    if (check_num_ < check_thres_)
                    {
                        armor_checker.run(current_frame_, robot_fgmask, enemy_rois, ENEMY_COLOR); //输入fgmask图片，输出ROI矩形框,分辨机器人颜色,在ROI区域中查找装甲板并输出装甲板ROI
//TODO: move it inside class
#ifdef SHOW_IMAGE
                        //画出方机器人的位置
                        for (int i = 0; i < enemy_rois.size(); i++)
                        {
                            if (ENEMY_COLOR)
                            { //画出对应颜色机器人
                                rectangle(frame_show, enemy_rois[i], Scalar(255, 0, 0), 2);
                            }
                            else
                            {
                                rectangle(frame_show, enemy_rois[i], Scalar(0, 0, 255), 2);
                            }
                        }
#endif
                    }
                    robot_tracker.run(current_frame_, check_num_, enemy_rois, enemy_tracker); //将ROI矩形框输入跟踪器进行跟踪
//TODO: add data
#ifdef USE_SERIAL
                    serial_port.sendData(send_pack_);
#ifdef PLOT_DATA
                    plot_serial.sendData(plot_pack_);
#endif
#endif

#ifdef SHOW_IMAGE
                    //画出方机器人的位置
                    if (!enemy_tracker->getObjects().empty())
                    {
                        for (int i = 0; i < enemy_rois.size(); i++)
                        {
                            if (ENEMY_COLOR)
                            { //画出对应颜色机器人
                                rectangle(frame_show, enemy_tracker->getObjects()[i], Scalar(0, 255, 0), 2);
                            }
                            else
                            {
                                rectangle(frame_show, enemy_tracker->getObjects()[i], Scalar(0, 255, 0), 2);
                            }
                        }
                    }
                    cout << "framecnt :" << frame_count << endl;
                    imshow("final result", frame_show);
                    waitKey(5);

#ifdef RUNNING_TIME
                    cout << "frame total time: " << timer_frame.getTime() << "ms\n";
                    timer_frame.stop();
#endif
                }
#if TEST == 1
                if (waitKey(0) == 27)
                    exit(0);
#elif TEST == 2
                if (waitKey(30) == 27)
                    break;
#endif
#endif
            }
        }
        catch (SerialException &e)
        {
            // 因已在msg_comm_thread线程中作了串口重启，
            // 为防止重启冲突造成程序bug，这里只接异常而不处理
        }
    }
}

void Workspace::misImgProcFunc()
{
}

void Workspace::msgCommFunc()
{
    while (true)
    {
        try
        {
            serial_port.readData(read_pack_);
        }
        catch (const SerialException &e)
        {
            cout << "Serial port read error." << endl;
            sleep(1);
            for (int i = 0; i < 5; ++i)
            {
                try
                {
                    openSerialPort();
                    if (serial_port.isOpen())
                        break;
                }
                catch (SerialException &e2)
                {
                    cout << "Try to open serial port error." << endl;
                    sleep(1);
                }
            }
            if (!serial_port.isOpen())
                exit(1);
        }
    }
}

void Workspace::openSerialPort()
{
// plot serial needs opening just once
#ifdef PLOT_DATA
    //TODO: aliases for both seiral port
    string plot_serial_name = "";
    plot_serial.open(plot_serial_name);
    if (plot_serial.isOpen())
    {
        cout << "Open plot serial " + plot_serial_name + "successfully.\n";
    }
    else
    {
        cerr << "Open plot serial " + plot_serial_name + " failed.\n";
        exit(1);
    }
#endif
    string serial_name = "";
    int fail_count = 0;
    while (fail_count < 3)
    {
        try
        {
            serial_port.open(serial_name);
            if (serial_port.isOpen())
            {
                cout << "Open serial " + serial_name + "successfully.\n";
                return;
            }
        }
        catch (const SerialException &e)
        {
            cout << e.what() << endl;
        }
    }
    throw SerialException("Open serial failed.");
}
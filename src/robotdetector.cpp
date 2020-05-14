/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Zeng Jing on 2020.04.08.

 Update:

 Detail:
 *****************************************************************************/

#include "robotdetector.h"

RobotDetector::RobotDetector() {

}

RobotDetector::~RobotDetector() {

}

void RobotDetector::init(const FileStorage &file_storage) {
    robot_background = createBackgroundSubtractorKNN(history_num, dist2_thres, false);//knn背景提取类创建
//history num -- 用于建模的历史帧数（可调参）
//dist2Thres -- 像素和样本之间平方距离的阈值，以确定像素是否接近该样本
//detectShadow -- 如果为true，算法将检测阴影并标记它们。
// 它会降低速度，因此如果您不需要此功能，请将参数设置为false。
//(此处默认不需要检测阴影）
}


void RobotDetector::run(const Mat &frame,Mat &fgmask) {
#ifdef RUNNING_TIME
    Timer timer_robot;
    timer_robot.start();
#endif

#ifdef SHOW_IMAGE
    imshow("frame", frame);
    waitKey(2);
#endif
    cvtColor(frame, gray_image, CV_BGR2GRAY);
    robot_background->apply(gray_image, fgmask);
    //将视频原图灰度化后输入knn背景提取类中，得到前景提取结果（前景为白背景为黑的黑白掩码图）
#ifdef SHOW_IMAGE
    imshow("original FGMask_KNN", fgmask);//展示前景提取结果
    waitKey(2);
#endif
#ifdef RUNNING_TIME
    cout << "detect robot time : " << timer_robot.getTime() << " ms " << endl;
    timer_robot.stop();
#endif
}





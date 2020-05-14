/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Zeng Jing on 2020.04.08.

 Update: Bruce Hou on 2020.04.30

 Detail:
 *****************************************************************************/

#include "armorchecker.h"
#include <iostream>


ArmorChecker::ArmorChecker() {

}

ArmorChecker::~ArmorChecker() {

}

void ArmorChecker::init(FileStorage file_storage) {
    armor_detector.init(file_storage);
}

void ArmorChecker::run(const Mat &src,Mat &fgmask, vector<Rect> &enemy_rois, int enemy_color) {
#ifdef RUNNING_TIME
    Timer timer;
    timer.start();
#endif
    vector<Rect> rois;

    preprocessFGmask(fgmask);//对前景提取结果进行腐蚀（去噪点）、膨胀（扩大区域框出机器人）
    findRobot(fgmask, src, rois);//通过前景提取结果找出机器人并框出ROI区域
    if (rois.empty()) {
        cout << "未检测到机器人" << endl;
    } else {
        checkArmor(src, rois, enemy_rois, enemy_color);
    }
#ifdef RUNNING_TIME
    cout << "check armor time:" << timer.getTime() << "ms" << endl;
    timer.stop();
#endif
}

void ArmorChecker::preprocessFGmask(Mat &fgmask) {
#ifdef RUNNING_TIME
    Timer preprocess_timer;
    preprocess_timer.start();
#endif  // RUNNING_TIME

    mask_thres = mean(fgmask).val[0];
    cv::threshold(fgmask, fgmask, mask_thres, 255.0, CV_THRESH_BINARY);
    //执行二值化处理，获得二值化掩码图

    erode(fgmask, fgmask, erode_kernel);
    //腐蚀处理，以消除噪音（可消除与目标大小不同的噪音）
#ifdef SHOW_IMAGE
    //imshow("erode FGMask_KNN", fgmask);//展示腐蚀结果
    //waitKey(2);
#endif

    cv::dilate(fgmask, fgmask, dilate_kernel);//膨胀处理，扩大机器人区域
#ifdef SHOW_IMAGE
    imshow("dilate FGMask_KNN", fgmask);
    waitKey(2);//展示掩码图处理最终结果
#endif

#ifdef RUNNING_TIME
    cout << "preprocess fgmask time: " << preprocess_timer.getTime() << "ms" << endl;
    preprocess_timer.stop();
#endif  // RUNNING_TIME
}

void ArmorChecker::findRobot(const Mat &fgmask, const Mat &src, vector<Rect> &rois) {
#ifdef RUNNING_TIME
    Timer find_timer;
    find_timer.start();
#endif  // RUNNING_TIME

    findContours(fgmask, robot_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//查找图片中所有机器人轮廓（并画出）
#ifdef SHOW_IMAGE
    Mat src_show = src.clone();
    drawContours(src_show, robot_contours, -1, Scalar(0, 255, 0), 1, 8);
    //imshow("contours",src_show);//展示轮廓绘制结果
#endif

    int i;
    for (i = 0; i < robot_contours.size(); i++) {//遍历并框出机器人轮廓
        robot_roi = boundingRect(Mat(robot_contours[i]));
        if (robot_roi.height > 0 && robot_roi.height < src.size().height
            && robot_roi.width > 0 && robot_roi.width < src.size().width) {//对ROI矩形边长进行筛选，防止报错
            robot_roi = correctROI(robot_roi, src);//防止ROI区域超出原图范围
#ifdef SHOW_IMAGE
            rectangle(src_show, robot_roi, Scalar(0, 255, 0), 2);
#endif
            rois.push_back(robot_roi);
        }
    }
#ifdef SHOW_IMAGE
    imshow("robot result", src_show);
    waitKey(2);//展示前景提取查找结果
#endif
#ifdef RUNNING_TIME
    cout << "find robot time: " << find_timer.getTime() << "ms" << endl;
    find_timer.stop();
#endif  // RUNNING_TIME
}

void ArmorChecker::checkArmor(const Mat &image, vector<Rect> rois, vector<Rect> &enemy_rois, int enemy_color) {
    Mat robot_image, armor_image;
    RotatedRect armor;
    enemy_rois.clear();
    for (int i = 0; i < rois.size(); i++) {
        if (rois[i].width > 0 && rois[i].width < image.size().width && rois[i].height > 0 &&
            rois[i].height < image.size().height) {
            image(rois[i]).copyTo(robot_image);
            armor_detector.run(robot_image, enemy_color, armor);
            Rect armor_roi = armor.boundingRect();
            if (armor_roi.width > 0 && armor_roi.width < robot_image.size().width && armor_roi.height > 0 &&
                armor_roi.height < robot_image.size().height) {
                robot_image(armor_roi).copyTo(armor_image);

                //数字识别部分预留（对ROI图进行检测）

                enemy_rois.emplace_back(rois[i]);
                cout << "检测到机器人" << endl;
            } else {
                cout << "未检测到装甲版,该目标非机器人" << endl;
            }
        }
    }
}

Rect correctROI(Rect original_roi, const Mat &frame) {
    //防止ROI区域超出原图范围
    int roi_rect_x = original_roi.tl().x;
    int roi_rect_y = original_roi.tl().y;
    int roi_rect_w = original_roi.width;
    int roi_rect_h = original_roi.height;
    if (roi_rect_x < 0) roi_rect_x = 0;
    if (roi_rect_y < 0) roi_rect_y = 0;
    if (roi_rect_x + roi_rect_w > frame.size().width) {
        roi_rect_w = roi_rect_w - (roi_rect_x + roi_rect_w - frame.size().width);
    }
    if (roi_rect_y + roi_rect_h > frame.size().height) {
        roi_rect_h = roi_rect_h - (roi_rect_y + roi_rect_h - frame.size().height);
    }
    Rect correct_roi = Rect(roi_rect_x, roi_rect_y, roi_rect_w, roi_rect_h);
    return correct_roi;
}



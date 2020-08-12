/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Zeng Jing on 2020.04.09.

 Update: Bruce Hou on 2020.04.30.

 Detail:
 *****************************************************************************/
#include "robottracker.h"
#include <iostream>


RobotTracker::RobotTracker() {

}

RobotTracker::~RobotTracker() {

}

void RobotTracker::run(const Mat &src, int &enemy_check_cnt,vector<Rect> &enemy_rois, Ptr<MultiTracker> &enemy_tracker) {
#ifdef RUNNING_TIME
    Timer timer_tracker;
    timer_tracker.start();
#endif

    cout << "敌方机器人情况 :" << endl;
    check(enemy_target_num, enemy_rois.size(), enemy_tracker, enemy_check_cnt);
    track(src, enemy_check_cnt, enemy_rois, enemy_tracker, enemy_target_num);
#ifdef RUNNING_TIME
    cout << "tracker time : " << timer_tracker.getTime() << " ms " << endl;
    timer_tracker.stop();
#endif
}

void RobotTracker::check(int &target_num, int roi_num, Ptr<MultiTracker> &multitracker, int &check_cnt) {
    if (target_num != roi_num) {//当记录的目标数与当前帧同色ROI矩形个数相等时，视为检测稳定
        cout << "机器人检测不稳定，计数归零！" << endl;
        multitracker.release();
        multitracker = cv::MultiTracker::create();
        target_num = roi_num;
        check_cnt = 0;//检测不稳定时，重设多目标跟踪类、目标数和检测计数变量
    } else {
        cout << "机器人正在检测中......" << endl;
        check_cnt++;
    }
}

void RobotTracker::track(const Mat &image, int &check_cnt, vector<Rect> &rois, Ptr<MultiTracker> &multitracker,
                         int &target_num) {
    if (check_cnt == check_thres) {//一旦检测稳定帧数达到阈值，开始跟踪
        cout << "机器人检测稳定，开始跟踪！" << endl;
        if (rois.empty()) {
            cout << "未检测到机器人！" << endl;
        } else {

            for (int i = 0; i < rois.size(); i++) {//遍历机器人ROI矩形
                if (rois[i].height > 0 && rois[i].height < image.size().height
                    && rois[i].width > 0 && rois[i].width < image.size().height) {//ROI矩形没有出错
                    Ptr<Tracker> tracker = TrackerKCF::create();//创建KCF算法的单跟踪器
                    multitracker->add(tracker, image, Rect2d(rois[i]));//将单跟踪器、当前帧和ROI矩形输入多目标跟踪器
                }
            }
        }
    }

    if (check_cnt > check_thres) {//检测稳定帧数达到阈值后，计数变量用来计算跟踪帧数
        //用最新帧更新跟踪器
        bool ok = multitracker->update(image);
        if (ok && check_cnt <= track_thres) {//正常跟踪（跟踪状态稳定，跟踪帧数小于一定值）
            cout << "机器人跟踪稳定！" << endl;
            check_cnt++;
        } else {//跟踪失败或跟踪帧数超过阈值，重新跟踪，重设多目标跟踪器和检测计数变量
            cout << "机器人跟踪失败，重新跟踪！" << endl;
            multitracker.release();
            multitracker = cv::MultiTracker::create();
            check_cnt = 0;
        }
    }
}

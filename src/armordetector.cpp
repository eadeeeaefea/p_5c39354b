/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#include "armordetector.h"
#include <iostream>
#include <cmath>


ArmorDetector::ArmorDetector() {

}

ArmorDetector::~ArmorDetector() {

}

void ArmorDetector::init(const FileStorage &file_storage) {
    // preprocess
#ifdef DISTORTION_CORRECT
    file_storage["camera_matrix"] >> camera_matrix_;
    file_storage["distortion_coeff"] >> distortion_coeff_;
#endif
    gray_thres_ = 20;
    subtract_thres_ = 20;
    kernel_size_ = 3;
#ifndef COMPILE_WITH_CUDA
    kernel_ = getStructuringElement(MORPH_RECT, Size(kernel_size_,kernel_size_));
#else
    kernel_ = cv::cuda::createMorphologyFilter(MORPH_OPEN, CV_8U,
        getStructuringElement(MORPH_RECT, Size(kernel_size_,kernel_size_)));
#endif

    // find armors
    min_ratio_ = 1.4;
    max_ratio_ = 5.6;
    min_len_ratio_ = 0.5;
    max_len_ratio_ = 2.0;
    max_lightbar_angle_ = 20.0;  // 下面三个tracebar上为*10
    max_armor_angle_ = 40.0;
    max_armor_lightbar_delta_ = 20.0;
}

void ArmorDetector::run(const Mat &src,
                        const int enemy_color,
                        RotatedRect &target_armor) {
#ifdef TRACKBAR
    namedWindow("processed", 1);
    createTrackbar("gray_thres", "processed", &gray_thres_, 60, 0, 0);
    createTrackbar("subtract_thres", "processed", &subtract_thres_, 60, 0, 0);
    createTrackbar("kernel_size", "processed", &kernel_size_, 15, 0, 0);
#endif
#ifdef SHOW_IMAGE
    src.copyTo(original_image_);
#endif
    Preprocess(src, enemy_color, processed_image_);
#ifdef TRACKBAR
    namedWindow("original", 1);

    static int min_ratio = static_cast<int>(min_ratio_ * 100.0);
    createTrackbar("min_ratio", "original", &min_ratio, 250, 0, 0);
    min_ratio_ = static_cast<double>(min_ratio) / 100;

    static int max_ratio = static_cast<int>(max_ratio_ * 100.0);
    createTrackbar("max_ratio", "original", &max_ratio, 600, 0, 0);
    max_ratio_ = static_cast<double>(max_ratio) / 100;

    static int min_len_ratio = static_cast<int>(min_len_ratio_ * 100.0);
    createTrackbar("min_len_ratio", "original", &min_len_ratio, 90, 0, 0);
    min_len_ratio_ = static_cast<double>(min_len_ratio) / 100.0;

    static int max_len_ratio = static_cast<int>(max_len_ratio_ * 100.0);
    createTrackbar("max_len_ratio", "original", &max_len_ratio, 300, 0, 0);
    max_len_ratio_ = static_cast<double>(max_len_ratio) / 100;

    static int max_lightbar_angle = static_cast<int>(max_lightbar_angle_ * 10.0);
    createTrackbar("max_lightbar_angle", "original", &max_lightbar_angle, 500, 0, 0);
    max_lightbar_angle_ = static_cast<double>(max_lightbar_angle) / 10;

    static int max_armor_angle = static_cast<int>(max_armor_angle_ * 10.0);
    createTrackbar("max_armor_angle", "original", &max_armor_angle, 500, 0, 0);
    max_armor_angle_ = static_cast<double>(max_armor_angle) / 10;

    static int max_armor_lightbar_delta = static_cast<int>(max_armor_lightbar_delta_ * 10.0);
    createTrackbar("max_armor_lightbar_delta", "original", &max_armor_lightbar_delta, 500, 0, 0);
    max_armor_lightbar_delta_ = static_cast<double>(max_armor_lightbar_delta) / 10;
#endif
    findTarget(processed_image_, target_armor);
#ifdef SHOW_IMAGE
    Rect rect = target_armor.boundingRect();
    rectangle(original_image_, rect, Scalar(0,255,0), 2);
    imshow("original", original_image_);
#ifdef TRACKBAR
    copyMakeBorder(processed_image_, processed_image_, 0, 480-processed_image_.rows,
                   0, 640-processed_image_.cols, BORDER_CONSTANT, Scalar(255,255,255));
#endif
    imshow("processed", processed_image_);
#ifdef ROI_ENABLE
    imshow("roi", roi_image_);
#endif
#endif
}

void ArmorDetector::Preprocess(const Mat &src,
                               const int enemy_color,
                               Mat &processed_image) {
#ifdef RUNNING_TIME
    Timer timer;
    timer.start();
#endif

#ifdef ROI_ENABLE
    if (roi_rect_.empty()) {
        src.copyTo(roi_image_);
    } else {
        src(roi_rect_).copyTo(roi_image_);
    }
#else
    src.copyTo(roi_image_);
#endif

#ifndef COMPILE_WITH_CUDA  // cpu only
#ifdef DISTORTION_CORRECT
    initUndistortRectifyMap(camera_matrix_, distortion_coeff_, Mat(),
        getOptimalNewCameraMatrix(camera_matrix_, distortion_coeff_, roi_image_.size(), 1, roi_image_.size(), 0),
        roi_image_.size(), CV_16SC2, map1_, map2_);
    remap(roi_image_, roi_image_, map1_, map2_, INTER_LINEAR);
#endif
    cvtColor(roi_image_, gray_image_, COLOR_BGR2GRAY);
    threshold(gray_image_, gray_image_, gray_thres_, 255, THRESH_BINARY);

    split(roi_image_, channels_);
    if (enemy_color) {
        subtract(channels_[0], channels_[2], subtract_image_);
    } else {
        subtract(channels_[2], channels_[0], subtract_image_);
    }
    threshold(subtract_image_, subtract_image_, subtract_thres_, 255, THRESH_BINARY);

    processed_image = gray_image_ & subtract_image_;
#ifdef SHOW_IMAGE
    kernel_ = getStructuringElement(MORPH_RECT, Size(kernel_size_,kernel_size_));
#endif
    morphologyEx(processed_image, processed_image, MORPH_OPEN, kernel_);

#else  // use gpu
#ifdef DISTORTION_CORRECT
    initUndistortRectifyMap(camera_matrix_, distortion_coeff_, Mat(),
        getOptimalNewCameraMatrix(camera_matrix_, distortion_coeff_, roi_image_.size(), 1, roi_image_.size(), 0),
        roi_image_.size(), CV_16SC2, map1_, map2_);
    gpu_map1_.upload(map1_);
    gpu_map2_.upload(map2_);
    cv::cuda::remap(roi_image_, roi_image_, gpu_map1_, gpu_map2_, INTER_LINEAR);
#endif
    gpu_src_.upload(roi_image_);

    cv::cuda::cvtColor(gpu_src_, gray_image_, COLOR_BGR2GRAY);
    cv::cuda::threshold(gray_image_, gray_image_, gray_thres_, 255, THRESH_BINARY);

    cv::cuda::split(gpu_src_, channels_);
    if (enemy_color) {
        cv::cuda::subtract(channels_[0], channels_[2], subtract_image_);
    } else {
        cv::cuda::subtract(channels_[2], channels_[0], subtract_image_);
    }
    cv::cuda::threshold(subtract_image_, subtract_image_, subtract_thres_, 255, THRESH_BINARY);

    cv::cuda::bitwise_and(gray_image_, subtract_image_, gpu_dst_);
#ifdef SHOW_IMAGE
    kernel_ = cv::cuda::createMorphologyFilter(MORPH_OPEN, CV_8U,
        getStructuringElement(MORPH_RECT, Size(kernel_size_,kernel_size_)));
#endif
    kernel_->apply(gpu_dst_, gpu_dst_);
    gpu_dst_.download(processed_image);
#endif

#ifdef RUNNING_TIME
    cout << "preprocess time: " << timer.getTime() << "ms" << endl;
    timer.stop();
#endif
}

void ArmorDetector::findTarget(const Mat &processed_image, RotatedRect &target_armor) {
#ifdef RUNNING_TIME
    Timer timer;
    timer.start();
#endif
    lightbars_.clear();
    findContours(processed_image, contours_, hierarchy_, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours_.size(); ++i) {
        temp_rrect_ = minAreaRect(contours_[i]);
        if (temp_rrect_.size.width * temp_rrect_.size.height > 10) {
            lightbars_.push_back(temp_rrect_);
        }
    }

    findArmors(lightbars_, armors_, scores_);
    selectTarget(armors_, scores_, target_armor);

#ifdef ROI_ENABLE
    if (!target_armor.size.empty()) {
        if (!roi_rect_.empty()) {
            target_armor.center.x += roi_rect_.x;
            target_armor.center.y += roi_rect_.y;
        }
        set_roi_rect(target_armor.boundingRect());
    } else {
        roi_rect_ = Rect();
    }
#endif
#ifdef RUNNING_TIME
    cout << "find target time: " << timer.getTime() << "ms" << endl;
    timer.stop();
#endif
}

void ArmorDetector::findArmors(vector<RotatedRect> &lightbars,
                               vector<RotatedRect> &armors,
                               vector<double> &scores) {
    armors.clear();
    scores.clear();
    if (lightbars.empty() || lightbars.size() == 1)    return;

    for (int i = 0; i < lightbars.size(); ++i) {
        adjustRotatedRect(lightbars[i]);
    }

    subscript_.clear();
    for (int i = 0; i < lightbars.size() - 1; ++i) {
        left_center_ = lightbars[i].center;

        for (int j = i + 1; j < lightbars.size(); ++j) {
            right_center_ = lightbars[j].center;

            // 装甲板宽高比
            armor_width_ = sqrt((right_center_.x - left_center_.x) *
                                (right_center_.x - left_center_.x) +
                                (right_center_.y - left_center_.y) *
                                (right_center_.y - left_center_.y));
            armor_height_ = (lightbars[i].size.height + lightbars[j].size.height) / 2;
            current_ratio_ = armor_width_ / armor_height_;

            // 两灯条长度比
            length_ratio_ = lightbars[i].size.height / lightbars[j].size.height;

            // 灯条倾斜度之差
            lightbar_angle_delta_ = fabs(lightbars[i].angle - lightbars[j].angle);

            // 装甲板倾斜度
            armor_angle_ = atan((right_center_.y - left_center_.y) /
                                (right_center_.x - left_center_.x)) * 180 / PI;

            // 灯甲倾斜度之差
            armor_lightbar_delta_ = fabs((lightbars[i].angle + lightbars[j].angle) / 2.0 - armor_angle_);

            // 判断阶段
            temp_result_ = 1;
            temp_score_ = 0.0;
            // 长宽比在范围内
            temp_result_ = temp_result_ && current_ratio_ > min_ratio_ && current_ratio_ < max_ratio_;
            temp_score_ += fabs(current_ratio_ - 3.31) * 10.0 / 3.31;  // 标准宽高比和其权值
            // cout << temp_result_ << "  " << temp_score_ << endl;
            // 两灯条长度相差不过大
            temp_result_ = temp_result_ && length_ratio_ > min_len_ratio_ && length_ratio_ < max_len_ratio_;
            if (length_ratio_ > 1.0) {
                temp_score_ += (length_ratio_ - 1.0) * 10.0 / (max_len_ratio_ - 1.0);  // 权值
            } else {
                temp_score_ += (1.0 - length_ratio_) * 10.0 / (1.0 - min_len_ratio_);  // 权值
            }
            // cout << temp_result_ << "  " << temp_score_ << endl;
            // 两灯条倾斜度之差不过大
            temp_result_ = temp_result_ && lightbar_angle_delta_ < max_lightbar_angle_;
            temp_score_ += lightbar_angle_delta_ * 10.0 / max_lightbar_angle_;  // 权值
            // cout << temp_result_ << "  " << temp_score_ << endl;
            // 装甲板不过于倾斜
            temp_result_ = temp_result_ && fabs(armor_angle_) < max_armor_angle_;
            temp_score_ += fabs(armor_angle_) * 10.0 / max_armor_angle_;  // 权值
            // cout << temp_result_ << "  " << temp_score_ << endl;
            // 灯甲倾斜度相差不过大
            temp_result_ = temp_result_ && armor_lightbar_delta_ < max_armor_lightbar_delta_;
            temp_score_ += armor_lightbar_delta_ * 10.0 / max_armor_lightbar_delta_;  // 权值
            // cout << temp_result_ << "  " << temp_score_ << endl;

            if (temp_result_) {
                subscript_.push_back(i);
                subscript_.push_back(j);
                scores.push_back(temp_score_);
            }
        }
    }

    for (int i = 0; i < subscript_.size(); i += 2) {
        lightbars[subscript_[i]].points(left_vertices_);
        lightbars[subscript_[i+1]].points(right_vertices_);
        for (int j = 0; j < 4; ++j) {
            armor_vertices_.push_back(left_vertices_[j]);
            armor_vertices_.push_back(right_vertices_[j]);
        }
        temp_rrect_ = minAreaRect(armor_vertices_);
        armors.push_back(temp_rrect_);
        armor_vertices_.clear();
    }
}

void ArmorDetector::selectTarget(const vector<RotatedRect> &armors,
                                 const vector<double> &scores,
                                 RotatedRect &target_armor) {
    target_armor = RotatedRect();
    if (armors.empty() || scores.empty())    return;

    int subscript;
    double min_score = 99999.9;
    for (int i = 0; i < scores.size(); ++i) {
        if (scores[i] < min_score) {
            min_score = scores[i];
            subscript = i;
        }
    }

    target_armor = armors[subscript];
}

void ArmorDetector::adjustRotatedRect(RotatedRect &rect) {
    if (rect.size.width > rect.size.height) {
        rect = RotatedRect(rect.center, Size2f(rect.size.height,rect.size.width), rect.angle+90);
    }
}

void ArmorDetector::preventROIExceed(int &x, int &y, int &width, int &height) {
    if (x < 0)    x = 0;
    if (y < 0)    y = 0;
    if (x + width > 640)    width = width - (x + width - 640);
    if (y + height > 480)   height = height - (y + height - 480);
}

#ifdef ROI_ENABLE
Rect ArmorDetector::get_roi_rect() {
    return roi_rect_;
}

void ArmorDetector::set_roi_rect(const Rect &rect) {
    int detect_x, detect_y;
    int detect_width, detect_height;

    detect_width = static_cast<int>(rect.width * 2.5);
    detect_height = static_cast<int>(rect.height * 4.0);
    detect_x = rect.x - (detect_width - rect.width) / 2;
    detect_y = rect.y - (detect_height - rect.height) / 2;

    preventROIExceed(detect_x, detect_y, detect_width, detect_height);

    roi_rect_ = Rect(detect_x, detect_y, detect_width, detect_height);
}
#endif

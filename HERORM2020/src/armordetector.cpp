/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Update: Wang Xiaoyan on 2019.12.19

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
#ifdef DISTORTION_CORRECT
    file_storage["camera_matrix"] >> camera_matrix;
    file_storage["distortion_coeff"] >> distortion_coeff;
#endif
#ifdef BGR
    gray_thres = 20;
    subtract_thres = 40;
    kernel_size = 3;
#endif
#ifdef HSV
    minH_red = 0;    maxH_red = 180;
    minS_red = 200;  maxS_red = 255;
    minV_red = 40;   maxV_red = 180;
    minH_blue = 60;  maxH_blue = 150;
    minS_blue = 100; maxS_blue = 200;
    minV_blue = 200; maxV_blue = 255;
    kernel_size = 5;
#endif
#ifndef COMPILE_WITH_CUDA
    kernel = getStructuringElement(MORPH_RECT, Size(kernel_size,kernel_size));
#else
    kernel = cv::cuda::createMorphologyFilter(MORPH_CLOSE, CV_8U,
        getStructuringElement(MORPH_RECT, Size(kernel_size,kernel_size)));
#endif

    min_aspect_ratio = 2.0;
    max_aspect_ratio = 6.0;
    min_length_ratio = 0.5;
    max_length_ratio = 2.0;
    max_lightbar_delta = 20.0;
    max_armor_angle = 30.0;
    max_armor_lightbar_delta = 20.0;
}

void ArmorDetector::run(const Mat &src,
                        const int enemy_color,
                        RotatedRect &target_armor) {
#ifdef TRACKBAR
    namedWindow("processed", 1);
#ifdef BGR
    createTrackbar("gray_thres", "processed", &gray_thres, 60, 0, 0);
    createTrackbar("subtract_thres", "processed", &subtract_thres, 60, 0, 0);
#endif  // BGR
#ifdef HSV
    if (enemy_color) {
        createTrackbar("min_H", "processed", &minH_blue, 255, 0, 0);
        createTrackbar("max_H", "processed", &maxH_blue, 255, 0, 0);
        createTrackbar("min_S", "processed", &minS_blue, 255, 0, 0);
        createTrackbar("max_S", "processed", &maxS_blue, 255, 0, 0);
        createTrackbar("min_V", "processed", &minV_blue, 255, 0, 0);
        createTrackbar("max_V", "processed", &maxV_blue, 255, 0, 0);
    } else {
        createTrackbar("min_H", "processed", &minH_red, 255, 0, 0);
        createTrackbar("max_H", "processed", &maxH_red, 255, 0, 0);
        createTrackbar("min_S", "processed", &minS_red, 255, 0, 0);
        createTrackbar("max_S", "processed", &maxS_red, 255, 0, 0);
        createTrackbar("min_V", "processed", &minV_red, 255, 0, 0);
        createTrackbar("max_V", "processed", &maxV_red, 255, 0, 0);
    }
#endif  // HSV
    createTrackbar("kernel_size", "processed", &kernel_size, 15, 0, 0);
#endif  // TRACKBAR
#ifdef SHOW_IMAGE
    static Mat original_image;
    src.copyTo(original_image);
#endif

    Preprocess(src, enemy_color, processed_image);

#ifdef TRACKBAR
    namedWindow("original", 1);

    static int min_asp_ratio = static_cast<int>(min_aspect_ratio * 100);
    createTrackbar("min_aspect_ratio", "original", &min_asp_ratio, 250, 0, 0);
    min_aspect_ratio = static_cast<double>(min_asp_ratio) / 100.0;

    static int max_asp_ratio = static_cast<int>(max_aspect_ratio * 100);
    createTrackbar("max_aspect_ratio", "original", &max_asp_ratio, 900, 0, 0);
    max_aspect_ratio = static_cast<double>(max_asp_ratio) / 100.0;

    static int min_len_ratio = static_cast<int>(min_length_ratio * 100);
    createTrackbar("min_length_ratio", "original", &min_len_ratio, 100, 0, 0);
    min_length_ratio = static_cast<double>(min_len_ratio) / 100.0;

    static int max_len_ratio = static_cast<int>(max_length_ratio * 100);
    createTrackbar("max_length_ratio", "original", &max_len_ratio, 400, 0, 0);
    max_length_ratio = static_cast<double>(max_len_ratio) / 100.0;

    static int max_lb_delta = static_cast<int>(max_lightbar_delta * 10);
    createTrackbar("max_lightbar_delta", "original", &max_lb_delta, 500, 0, 0);
    max_lightbar_delta = static_cast<double>(max_lb_delta) / 10.0;

    static int max_am_angle = static_cast<int>(max_armor_angle * 10);
    createTrackbar("max_armor_angle", "original", &max_am_angle, 500, 0, 0);
    max_armor_angle = static_cast<double>(max_am_angle) / 10.0;

    static int max_delta = static_cast<int>(max_armor_lightbar_delta * 10);
    createTrackbar("max_armor_lightbar_delta", "original", &max_delta, 500, 0, 0);
    max_armor_lightbar_delta = static_cast<double>(max_delta) / 10.0;
#endif  // TRACKBAR

    findTarget(processed_image, target_armor);

#ifdef SHOW_IMAGE
    drawRotatedRect(original_image, target_armor);
    imshow("original", original_image);
#ifdef TRACKBAR
    copyMakeBorder(processed_image, processed_image, 0, 480-processed_image.rows,
                   0, 640-processed_image.cols, BORDER_CONSTANT, Scalar(255,255,255));
#endif  // TRACKBAR
    imshow("processed", processed_image);
#ifdef ROI_ENABLE
    imshow("roi", roi_image);
#endif // ROI_ENABLE
#endif  // SHOW_IMAGE
}

void ArmorDetector::Preprocess(const Mat &src,
                               const int enemy_color,
                               Mat &dst) {
#ifdef RUNNING_TIME
    Timer timer;
    timer.start();
#endif  // RUNNING_TIME

#ifdef ROI_ENABLE
    if (roi_rect.empty()) {
        src.copyTo(roi_image);
    } else {
        src(roi_rect).copyTo(roi_image);
    }
#else
    src.copyTo(roi_image);
#endif  // ROI_ENABLE

#ifndef COMPILE_WITH_CUDA  // cpu
#ifdef DISTORTION_CORRECT
    static Mat map1, map2;
    initUndistortRectifyMap(camera_matrix, distortion_coeff, Mat(),
        getOptimalNewCameraMatrix(camera_matrix, distortion_coeff, roi_image.size(), 1, roi_image.size(), 0),
        roi_image.size(), CV_16SC2, map1, map2);
    remap(roi_image, roi_image, map1, map2, INTER_LINEAR);
#endif // DISTORTION_CORRECT
#ifdef BGR
    static Mat gray_image, subtract_image;
    static vector<Mat> channels;

    cvtColor(roi_image, gray_image, COLOR_BGR2GRAY);
    threshold(gray_image, gray_image, gray_thres, 255, THRESH_BINARY);

    split(roi_image, channels);
    if (enemy_color) {
        subtract(channels[0], channels[2], subtract_image);
    } else {
        subtract(channels[2], channels[0], subtract_image);
    }
    threshold(subtract_image, subtract_image, subtract_thres, 255, THRESH_BINARY);

    dst = gray_image & subtract_image;
#endif // BGR
#ifdef HSV
    static Mat hsv_image;

    cvtColor(roi_image, hsv_image, COLOR_BGR2HSV);
    if (enemy_color) {
        inRange(hsv_image, Scalar(minH_blue,minS_blue,minV_blue), Scalar(maxH_blue,maxS_blue,maxV_blue), dst);
    } else {
        inRange(hsv_image, Scalar(minH_red,minS_red,minV_red), Scalar(maxH_red,maxS_red,maxV_red), dst);
    }
#endif  // HSV
#ifdef TRACKBAR
    kernel = getStructuringElement(MORPH_RECT, Size(kernel_size,kernel_size));
#endif  // TRACKBAR
    morphologyEx(dst, dst, MORPH_CLOSE, kernel);

#else
#ifdef DISTORTION_CORRECT
    static Mat map1, map2;
    static cv::cuda::GpuMat gpu_map1, gpu_map2;
    initUndistortRectifyMap(camera_matrix, distortion_coeff, Mat(),
        getOptimalNewCameraMatrix(camera_matrix, distortion_coeff, roi_image.size(), 1, roi_image.size(), 0),
        roi_image.size(), CV_16SC2, map1, map2);
    gpu_map1.upload(map1);
    gpu_map2.upload(map2);
    cv::cuda::remap(roi_image, roi_image, gpu_map1, gpu_map2, INTER_LINEAR);
#endif  // DISTORTION_CORRECT
    static cv::cuda::GpuMat gpu_src, gpu_dst;
    gpu_src.upload(roi_image);
#ifdef BGR
    static cv::cuda::GpuMat gpu_gray, gpu_subtract;
    static vector<cv::cuda::GpuMat> gpu_channels;

    cv::cuda::cvtColor(gpu_src, gpu_gray, COLOR_BGR2GRAY);
    cv::cuda::threshold(gpu_gray, gpu_gray, gray_thres, 255, THRESH_BINARY);

    cv::cuda::split(gpu_src, gpu_channels);
    if (enemy_color) {
        cv::cuda::subtract(gpu_channels[0], gpu_channels[2], gpu_subtract);
    } else {
        cv::cuda::subtract(gpu_channels[2], gpu_channels[0], gpu_subtract);
    }
    cv::cuda::threshold(gpu_subtract, gpu_subtract, subtract_thres, 255, THRESH_BINARY);

    cv::cuda::bitwise_and(gpu_gray, gpu_subtract, gpu_dst);
#endif  // BGR
#ifdef HSV
    static cv::cuda::GpuMat gpu_hsv;

    cv::cuda::cvtColor(gpu_src, gpu_hsv, COLOR_BGR2HSV);
    if (enemy_color) {
        cv::cuda::inRange(gpu_hsv, Scalar(minH_blue,minS_blue,minV_blue), Scalar(maxH_blue,maxS_blue,maxV_blue), gpu_dst);
    } else {
        cv::cuda::inRange(gpu_hsv, Scalar(minH_red,minS_red,minV_red), Scalar(maxH_red,maxS_red,maxV_red), gpu_dst);
    }
#endif  // HSV
#ifdef TRACKBAR
    kernel = cv::cuda::createMorphologyFilter(MORPH_CLOSE, CV_8U,
        getStructuringElement(MORPH_RECT, Size(kernel_size,kernel_size)));
#endif  // TRACKBAR
    kernel->apply(gpu_dst, gpu_dst);
    gpu_dst.download(dst);
#endif  // COMPILE_WITH_CUDA

#ifdef RUNNING_TIME
    cout << "preprocess time: " << timer.getTime() << "ms" << endl;
    timer.stop();
#endif  // RUNNING_TIME
}

void ArmorDetector::findTarget(const Mat &dst, RotatedRect &target_armor) {
#ifdef RUNNING_TIME
    Timer timer;
    timer.start();
#endif  // RUNNING_TIME

    static vector<vector<Point> > contours;
    static vector<Vec4i> hierarchy;
    static RotatedRect temp_rect;
    static vector<RotatedRect> lightbars;

    lightbars.clear();
    findContours(dst, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); ++i) {
        temp_rect = minAreaRect(contours[i]);
        if (temp_rect.size.width * temp_rect.size.height > 20) {
            lightbars.push_back(temp_rect);
        }
    }

    static vector<RotatedRect> armors;
    static vector<double> scores;
    findArmors(lightbars, armors, scores);
    selectTarget(armors, scores, target_armor);

#ifdef ROI_ENABLE
    if (!target_armor.size.empty()) {
        if (!roi_rect.empty()) {
            target_armor.center.x += roi_rect.x;
            target_armor.center.y += roi_rect.y;
        }
        set_roi_rect(target_armor.boundingRect());
    } else {
        roi_rect = Rect();
    }
#endif  // ROI_ENABLE

#ifdef RUNNING_TIME
    cout << "find target time: " << timer.getTime() << "ms" << endl;
#endif  // RUNNING_TIME
}

void ArmorDetector::findArmors(vector<RotatedRect> &lightbars,
                               vector<RotatedRect> &armors,
                               vector<double> &scores) {
    armors.clear();
    scores.clear();
    if (lightbars.empty() || lightbars.size() == 1)     return;

    for (int i = 0; i < lightbars.size(); ++i) {
        adjustRotatedRect(lightbars[i]);
    }

    static Point2f left_center, right_center;
    static double armor_width, armor_height, aspect_ratio, length_ratio;
    static double lightbar_delta, armor_angle, armor_lightbar_delta;
    static vector<double> subscript;
    static double score;
    static bool result;

    subscript.clear();
    for (int i = 0; i < lightbars.size() - 1; ++i) {
        left_center = lightbars[i].center;

        for (int j = i + 1; j < lightbars.size(); ++j) {
            right_center = lightbars[j].center;

            // 装甲板宽高比
            armor_width = sqrt((right_center.x - left_center.x) *
                               (right_center.x - left_center.x) +
                               (right_center.y - left_center.y) *
                               (right_center.y - left_center.y));
            armor_height = (lightbars[i].size.height + lightbars[j].size.height) / 2;
            aspect_ratio = armor_width / armor_height;

            // 两灯条长度比
            length_ratio = lightbars[i].size.height / lightbars[j].size.height;

            // 灯条倾斜度之差
            lightbar_delta = fabs(lightbars[i].angle - lightbars[j].angle);

            // 装甲板倾斜度
            armor_angle = atan((right_center.y - left_center.y) / (right_center.x - left_center.x)) * 180 / PI;

            // 灯甲倾斜度之差
            armor_lightbar_delta = fabs((lightbars[i].angle + lightbars[j].angle) / 2.0 - armor_angle);

            // 判断
            result = 1;
            score = 0.0;
            // 长宽比在范围内
            result = result && aspect_ratio > min_aspect_ratio && aspect_ratio < max_aspect_ratio;
            score += fabs(aspect_ratio - 3.31) * 10.0 / 3.31;
            // cout << aspect_ratio << "  " << result << "  " << score << "\n";
            // 两灯条长度相差不过大
            result = result && length_ratio > min_length_ratio && length_ratio < max_length_ratio;
            if (length_ratio > 1.0) {
                score += (length_ratio - 1.0) * 10.0 / (max_length_ratio - 1.0);
            } else {
                score += (1.0 - length_ratio) * 10.0 / (1.0 - min_length_ratio);
            }
            // cout << length_ratio << "  " << result << "  " << score << "\n";
            // 两灯条倾斜度之差不过大
            result = result && lightbar_delta < max_lightbar_delta;
            score += lightbar_delta * 10.0 / max_lightbar_delta;
            // cout << lightbar_delta << "  " << result << "  " << score << "\n";
            // 装甲板倾斜度不过大
            result = result && fabs(armor_angle) < max_armor_angle;
            score += fabs(armor_angle) * 10.0 / max_armor_angle;
            // cout << armor_angle << "  " << result << "  " << score << "\n";
            // 灯甲倾斜度相差不过大
            result = result && armor_lightbar_delta < max_armor_lightbar_delta;
            score += armor_lightbar_delta * 10.0 / max_armor_lightbar_delta;
            // cout << armor_lightbar_delta << "  " << result << "  " << score << "\n";

            if (result) {
                subscript.push_back(i);
                subscript.push_back(j);
                scores.push_back(score);
            }
        }
    }

    static Point2f left_vertices[4], right_vertices[4];
    static vector<Point2f> armor_vertices;
    static RotatedRect temp_rect;

    for (int i = 0; i < subscript.size(); i += 2) {
        lightbars[subscript[i]].points(left_vertices);
        lightbars[subscript[i+1]].points(right_vertices);
        for (int j = 0; j < 4; ++j) {
            armor_vertices.push_back(left_vertices[j]);
            armor_vertices.push_back(right_vertices[j]);
        }
        temp_rect = minAreaRect(armor_vertices);
        armors.push_back(temp_rect);
        armor_vertices.clear();
    }
}

void ArmorDetector::selectTarget(const vector<RotatedRect> &armors,
                                 const vector<double> &scores,
                                 RotatedRect &target_armor) {
     target_armor = RotatedRect();
     if (armors.empty() || scores.empty())     return;

     static int subscript;
     static double min_score;
     min_score = 1e+5;
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
    return roi_rect;
}

void ArmorDetector::set_roi_rect(const Rect &rect) {
    int detect_x, detect_y;
    int detect_width, detect_height;

    detect_width = static_cast<int>(rect.width * 2.5);
    detect_height = static_cast<int>(rect.height * 4.0);
    detect_x = rect.x - (detect_width - rect.width) / 2;
    detect_y = rect.y - (detect_height - rect.height) / 2;

    preventROIExceed(detect_x, detect_y, detect_width, detect_height);

    roi_rect = Rect(detect_x, detect_y, detect_width, detect_height);
}
#endif  // ROI_ENABLE

void drawRotatedRect(Mat &src, RotatedRect &rect) {
    Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; ++i)
        line(src, vertices[i], vertices[(i+1)%4], Scalar(0,255,0), 2, 8);
}

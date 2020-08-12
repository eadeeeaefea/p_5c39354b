/**
 * @file main.cpp
 * @brief
 * @details
 * @author Zeng Jing
 * @email
 * @update Bruce Hou on 2020.04.30
 * @update Lu Zhan on 2020.08.10
 * @version
 * @license 2020 HITWH HERO-Robomaster Group
 */

#include <torch/script.h>
#include <torch/torch.h>
#include "armorchecker.h"
#include <iostream>


class DigitalRecognition {
private:
    torch::jit::script::Module module;
public:
    DigitalRecognition() {
        module = torch::jit::load("../param/model.pt");
    }

    int matToDigital(cv::Mat &img) {
        // 正则化
        img.convertTo(img, CV_32FC1, 1.0f / 255.0f);

        // 模型用的是 28*28 的单通道灰度图
        cv::resize(img, img, cv::Size(28, 28));

        // 将 OpenCV 的 Mat 转换为 Tensor, 注意两者的数据格式
        // OpenCV: H*W*C 高度, 宽度, 通道数
        auto input_tensor = torch::from_blob(img.data, {1, img.cols, img.rows, 1});
        // Tensor: N*C*H*W 数量, 通道数, 高度, 宽度
        // 数字表示顺序
        input_tensor = input_tensor.permute({0, 3, 1, 2});

        // 添加数据
        std::vector<torch::jit::IValue> inputs;
        inputs.emplace_back(input_tensor);

        // 模型计算
        at::Tensor output = module.forward(inputs).toTensor();
        std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';

        // 输出分类的结果
        int ans = output.argmax(1).item().toInt();
        std::cout << "当前机器人编号: " << ans << std::endl;
    }
};


DigitalRecognition digitalRecognition;

ArmorChecker::ArmorChecker() {

}

ArmorChecker::~ArmorChecker() {

}

void ArmorChecker::init(FileStorage file_storage) {
    armor_detector.init(file_storage);
}

void ArmorChecker::run(const Mat &src, Mat &fgmask, vector<Rect> &enemy_rois, int enemy_color) {
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

    cv::dilate(fgmask, fgmask, dilate_kernel, cv::Point(-1,-1), 4);//膨胀处理，扩大机器人区域图片大小会影响参数的选取
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
    drawContours(src_show, robot_contours, -1, cv::Scalar(0, 255, 0), 1, 8);
    //imshow("contours",src_show);//展示轮廓绘制结果
#endif

    int i;
    for (i = 0; i < robot_contours.size(); i++) {//遍历并框出机器人轮廓
        robot_roi = boundingRect(Mat(robot_contours[i]));
        if (robot_roi.height > 0 && robot_roi.height < src.size().height
            && robot_roi.width > 0 && robot_roi.width < src.size().width) {//对ROI矩形边长进行筛选，防止报错
            robot_roi = correctROI(robot_roi, src);//防止ROI区域超出原图范围
#ifdef SHOW_IMAGE
            rectangle(src_show, robot_roi, cv::Scalar(0, 255, 0), 2);
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
//            drawRotatedRect(robot_image, armor);
#ifdef SHOW_IMAGE
            cv::imshow("image", robot_image);
#endif
            /// 截取数字roi，Rect绕中心缩放，缩放调参
            Rect armor_roi = rectCenterScale(armor.boundingRect(), Size(-10, 10));
            try{
                Mat number_image;
                robot_image(armor_roi).copyTo(number_image);
                cvtColor(number_image, number_image, COLOR_BGR2GRAY);
                threshold(number_image, number_image, 0, 255, THRESH_BINARY);
#ifdef SHOW_IMAGE
                imshow("number_image", number_image);
#endif
                // 数字识别部分预留（对ROI图进行检测）
#ifdef RUNNING_TIME
                Timer timer;
                timer.start();
#endif
                digitalRecognition.matToDigital(number_image);            // 数字识别结束
#ifdef RUNNING_TIME
                cout << "数字识别运行时间: " << timer.getTime() << "ms" << endl;
                timer.stop();
#endif
                enemy_rois.emplace_back(rois[i]);
                cout << "检测到机器人" << endl;
            } catch (...) {
                cout << "未检测到装甲版,该目标非机器人" << endl;
            }

//            if (armor_roi.width > 0 && armor_roi.width < robot_image.size().width && armor_roi.height > 0 &&
//                armor_roi.height < robot_image.size().height) {
//                robot_image(armor_roi).copyTo(armor_image);
//                // 数字识别部分预留（对ROI图进行检测）
//                digitalRecognition.matToDigital(armor_image);
//                // 数字识别结束
//
//                enemy_rois.emplace_back(rois[i]);
//                cout << "检测到机器人" << endl;
//            } else {
//                cout << "未检测到装甲版,该目标非机器人" << endl;
//            }
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


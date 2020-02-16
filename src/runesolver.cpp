/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Lu Zhan
 Update: Bruce Hou
 Detail:
 *****************************************************************************/
#include "runesolver.h"

RuneSolver::RuneSolver() {
    isFindCenter = false;
    isCalibrated = false;
    runesolver_centers.reserve(50);
    arrow_centers.reserve(50);
    angle_set.reserve(50);
    angle_array.reserve(50);
    direction = DIR_DEFAULT;
    mode = MODE_DEFAULT;
    object_Points.reserve(5);
//    writer.open("/home/stayalive/Documents/HERO/Hero2020_final/1.avi", 0, 25.0, Size(//));
}

RuneSolver::~RuneSolver() {

}

void RuneSolver::init(const FileStorage &file_storage) {
    file_storage["camera_matrix"] >> CAMERA_MATRIX;
    file_storage["distortion_coeff"] >> DISTCOEFFS;
}

void RuneSolver::run(const Mat &image, const int enemy_color, double v, double &send_pitch, double &send_yaw,
                     double read_pitch,
                     double read_yaw) {
    //    Timer clock;
    image.copyTo(src);
    shoot = false;
    isLoseAllTargets = false;
    isFindRuneSolver = false;
    isFindArrow = false;
    preprocess(enemy_color);
    //找能量板
    if (findRuneSolver()) {
        circle(src, target_RuneSolver.center, 1, Scalar(0, 255, 0), 1);
//        draw(src, target_RuneSolver);
    } else {
//        cout << "no RuneSolver" << endl;
    }
    //找箭头
    if (findArrow()) {
        circle(src, target_arrow.center, 1, Scalar(0, 255, 0), -1);
//        draw(src, target_arrow);
    } else {
//        cout << " no arrow" << endl;
    }
    if (!isFindRuneSolver || !isFindArrow) {
        isLoseAllTargets = true;
        cout << "丢失目标!\n";
        return;
    }
    //箭头区域和能量板是否匹配
    if (!match()) {
        cout << "不匹配\n";
        return;
    }
    solveRealPostiton(target_RuneSolver);
    real_point.x = translation_matrix.at<double>(0, 0) / 1000;
    real_point.y = (translation_matrix.at<double>(1, 0) - 51.4469) / 1000;
    real_point.z = (translation_matrix.at<double>(2, 0) + 140.7033) / 1000;
    anglesolver.run(real_point.x, real_point.y, real_point.z, v, send_pitch, send_yaw, read_pitch);

//    flight_time = anglesolver.get_flight_time(real_point.x, real_point.y, real_point.z, v, read_pitch);
//    cout << "flight time: " << flight_time << endl;


    hit_angle.x = static_cast<float>(read_pitch + send_pitch);
    hit_angle.y = static_cast<float>(send_yaw + read_yaw);
    updateQueue();
    if (direction == DIR_DEFAULT) {
        if (!solveDirection())
            return;
    } else {
        switch (direction) {
            case DIR_CW:
            case DIR_CCW:
                mode = MODE_BIG;
                break;
            case DIR_STATIC:
                mode = MODE_SMALL;
                break;
        }
    }
//    getangularvelocity();
//    cout << "angular_velocity:" << angular_velocity << endl;
    predict();
    send_pitch = predict_angle.x - read_pitch;
    send_yaw = predict_angle.y - read_yaw;

    if (isnan(predict_angle.x) || isnan(predict_angle.y)) {
        predict_angle.x = predict_angle.y = 0;
    } else {
        predict_angle.x += 100;
        predict_angle.y += 100;
    }

    hit_angle.x = (hit_angle.x + 100);
    hit_angle.y = (hit_angle.y + 100);

    if (isnan(send_pitch) || isnan(send_yaw)) {
        send_pitch = send_yaw = 0;
    }
#ifdef SHOW_IMAGE
    circle(src, hit_angle, 2, Scalar(255, 0, 0), 1, -1);
    circle(src, predict_angle, 2, Scalar(0, 255, 0), 1, -1);
    imshow("src", src);
//    if (waitKey(1) == 27) {
//        exit(0);
//    }
//    if (waitKey(1) < 0) {
//        waitKey();
//    }
    //    waitKey(30);
#endif
}

float RuneSolver::getDistance(Point2f p1, Point2f p2) {
    return static_cast<float>(sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0)));
}

void RuneSolver::preprocess(const int enemy_color) {
#ifndef COMPILE_WITH_CUDA
    vector<Mat> channels;
    split(src, channels);
    if (enemy_color) {
        threshold(channels[0] - channels[2], bin, 80, 255, THRESH_BINARY);
    } else {
        threshold(channels[2] - channels[0], bin, 80, 255, THRESH_BINARY);
    }
#else
    src_.upload(src);
    vector<cv::cuda::GpuMat> channels;
    cv::cuda::split(src_, channels);
    if (enemy_color) {
        cv::cuda::subtract(channels[0], channels[2], bin_);
    } else {
        cv::cuda::subtract(channels[2], channels[0], bin_);
    }
    cv::cuda::threshold(bin_, bin_, 80, 255, THRESH_BINARY);
    bin_.download(bin);
#endif
}

bool RuneSolver::findRuneSolver() {
    isFindRuneSolver = false;
    double area;
    float ratio;
    RotatedRect temp_rect;
#ifndef COMPILE_WITH_CUDA
    Mat temp_bin;
    bin.copyTo(temp_bin);

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(temp_bin, temp_bin, element, Point(-1, -1), 2);
#ifdef SHOW_IMAGE
    imshow("填充前", temp_bin);
#endif
    floodFill(temp_bin, Point(1, 1), Scalar(255), 0, FLOODFILL_FIXED_RANGE);
#ifdef SHOW_IMAGE
    imshow("填充后", temp_bin);
#endif

    element = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(temp_bin, temp_bin, element, Point(-1, -1), 3);
#ifdef SHOW_IMAGE
    imshow("复原后", temp_bin);
#endif
#else
    cv::cuda::GpuMat temp_bin_;
    cv::Mat temp_bin;
    temp_bin_.upload(bin);
    Mat element = getStructuringElement(MORPH_RECT, Size(1, 1));
    Ptr<cv::cuda::Filter> dilateFilter = cv::cuda::createMorphologyFilter(MORPH_DILATE, CV_8U, element, Point(-1, -1),
                                                                          2);
    dilateFilter->apply(temp_bin_, temp_bin_);
    temp_bin_.download(temp_bin);
#ifdef SHOW_IMAGE
    imshow("填充前", temp_bin);
#endif
    floodFill(temp_bin, Point(1, 1), Scalar(255), 0, FLOODFILL_FIXED_RANGE);
#ifdef SHOW_IMAGE
    imshow("填充后", temp_bin);
#endif
    temp_bin_.upload(temp_bin);
    element = getStructuringElement(MORPH_RECT, Size(5, 5));
    Ptr<cv::cuda::Filter> erodeFilter = cv::cuda::createMorphologyFilter(MORPH_ERODE, CV_8U, element, Point(-1, -1), 2);
    erodeFilter->apply(temp_bin_, temp_bin_);
    temp_bin_.download(temp_bin);
#ifdef SHOW_IMAGE
    imshow("复原后", temp_bin);
#endif
#endif
    vector<vector<Point>> contours;
    vector<Point> target_contour;       //目标轮廓
    findContours(temp_bin, contours, RETR_CCOMP, CHAIN_APPROX_NONE);
//    cout<<contours.size()<<'\n';

    //筛选目标的能量板区域
    for (size_t i = 0; i < contours.size(); ++i) {
//        drawContours(src, contours, i, Scalar(200, 100, 0), 1);
        area = contourArea(contours[i]);
//        cout<<"能量板面积:  "<<area<<'\n';
        //根据面积筛选
        if (area < MIN_RuneSolver_AREA || area > MAX_RuneSolver_AREA)
            continue;

        target_contour = contours[i];
        if (target_contour.size() <= 5)
            continue;
        temp_rect = fitEllipse(target_contour);
        temp_rect.size.height *= 0.75;
        temp_rect.size.width *= 0.75;
        //按宽高比筛选
        float width, height;
        width = max(temp_rect.size.width, temp_rect.size.height);
        height = min(temp_rect.size.width, temp_rect.size.height);
        ratio = width / height;
//        cout<<"能量板比例："<<ratio<<'\n';
        if (ratio < MIN_RuneSolver_RATIO || ratio > MAX_RuneSolver_RATIO)
            continue;

        //统一样式
        if (temp_rect.size.width > temp_rect.size.height) {
            temp_rect.size = Size2f(height, width);
            temp_rect.angle += 90;
        }
        target_RuneSolver = temp_rect;
        isFindRuneSolver = true;
        break;
    }
    return isFindRuneSolver;
}

bool RuneSolver::findArrow() {
    isFindArrow = false;
    double area;
    float ratio;
    RotatedRect temp_rect;
#ifndef COMPILE_WITH_CUDA
    Mat temp_bin;
    bin.copyTo(temp_bin);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(temp_bin, temp_bin, element, Point(-1, -1), 2);
#else
    cv::cuda::GpuMat temp_bin_;
    cv::Mat temp_bin;
    temp_bin_.upload(bin);
    Mat element = getStructuringElement(MORPH_RECT, Size(1, 1));
    Ptr<cv::cuda::Filter> dilateFilter = cv::cuda::createMorphologyFilter(MORPH_DILATE, CV_8U, element, Point(-1, -1),
                                                                          2);
    dilateFilter->apply(temp_bin_, temp_bin_);
    temp_bin_.download(temp_bin);
#endif
#ifdef SHOW_IMAGE
    imshow("箭头预处理后", temp_bin);
#endif
    vector<vector<Point>> contours;
    vector<Point> target_contour;       //目标轮廓
    findContours(temp_bin, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
//    cout << "箭头轮廓总数:  " << contours.size() << '\n';
    //筛选目标的箭头区域
    for (size_t i = 0; i < contours.size(); ++i) {
//        drawContours(src, contours, i, Scalar(200, 100, 0), 1);
        area = contourArea(contours[i]);
//        cout << "箭头面积: " << area << '\n';
        //根据面积筛选
        if (area < MIN_ARROW_AREA || area > MAX_ARROW_AREA)
            continue;

        target_contour = contours[i];
//        drawContours(src, contours, i, Scalar(200, 100, 0), 1);
        temp_rect = fitEllipse(target_contour);

        float width, height;
        width = max(temp_rect.size.width, temp_rect.size.height);
        height = min(temp_rect.size.width, temp_rect.size.height);
        ratio = width / height;

//        cout << "箭头比例：" << ratio << '\n';
        if (ratio < MIN_ARROW_RATIO || ratio > MAX_ARROW_RATIO)
            continue;

        //统一样式
        if (temp_rect.size.width > temp_rect.size.height) {
            temp_rect.size = Size2f(height, width);
            temp_rect.angle += 90;
        }
        target_arrow = temp_rect;
//        target_arrow.center = total;
        isFindArrow = true;
        break;
    }
//    cout << "-------------------------------------\n";
    return isFindArrow;
}

bool RuneSolver::match() {
    vector<Point2f> intersection;
    if (rotatedRectangleIntersection(target_arrow, target_RuneSolver, intersection) > 0)
        return true;
    else
        return false;
}

void RuneSolver::solveRealPostiton(const cv::RotatedRect aim) {
    int i, j;
    RotatedRect rect = aim;
    object_Points.clear();
    image_Points.clear();

    Point2f vertices[4];
    rect.points(vertices);
    float dis;
    Point2f temp;
    //按顺时针重新排列四个角点
    for (i = 0; i < 4; ++i)
        image_Points.emplace_back(vertices[i]);
    dis = static_cast<float>(sqrt(
            pow(image_Points[0].x - image_Points[1].x, 2) + pow(image_Points[0].y - image_Points[1].y, 2)));
    if (abs(dis - rect.size.height) > EXP) {
        temp = image_Points[0];
        image_Points.erase(image_Points.begin());
        image_Points.emplace_back(temp);
    }
    if (image_Points[0].x > image_Points[1].x) {
        temp = image_Points[0];
        image_Points.erase(image_Points.begin());
        image_Points.emplace_back(temp);
        temp = image_Points[0];
        image_Points.erase(image_Points.begin());
        image_Points.emplace_back(temp);
    }
    String text;
    for (i = 0; i < 4; ++i) {
        text = to_string(i);
        putText(src, text, image_Points[i], FONT_HERSHEY_SIMPLEX, 0.35, Scalar(255, 255, 255));
    }
    object_Points.emplace_back(Point3f(-RuneSolver_HALF_LENGTH, -RuneSolver_HALF_WIDTH, 0));
    object_Points.emplace_back(Point3f(RuneSolver_HALF_LENGTH, -RuneSolver_HALF_WIDTH, 0));
    object_Points.emplace_back(Point3f(RuneSolver_HALF_LENGTH, RuneSolver_HALF_WIDTH, 0));
    object_Points.emplace_back(Point3f(-RuneSolver_HALF_LENGTH, RuneSolver_HALF_WIDTH, 0));
    solvePnP(object_Points, image_Points, CAMERA_MATRIX, DISTCOEFFS, rotated_vector, translation_matrix, false,
             SOLVEPNP_EPNP);
//    cout << rotated_vector << "\n" << translation_matrix << "\n";
}

void RuneSolver::updateQueue() {
    if (angle_set.size() >= QUEUE_SIZE) {
        angle_set.erase(angle_set.begin());
    }
    angle_set.emplace_back(hit_angle);
}

bool RuneSolver::findCenter() {
    if (angle_set.size() < QUEUE_SIZE) {
        return false;
    }
    isFindCenter = false;
    isFindCenter = calculate_ellipse();
    if (!isFindCenter) {
        angle_set.pop_back();
    }
    return isFindCenter;
}

bool RuneSolver::calculate_ellipse() {
    int i;
    RotatedRect fit_rect = fitEllipse(angle_set);
    ellipse(src, fit_rect, Scalar(0, 0, 255), 1, 1);
    ellipse_center = fit_rect.center;
    ellipse_angle = fit_rect.angle;
    ellipse_Xaxis = fit_rect.size.width / 2.0;
    ellipse_Yaxis = fit_rect.size.height / 2.0;
//    Point2f vertices[4];
//    vector<Point2f> temp_points;
//    Point2f temp_point;
//    fit_rect.points(vertices);
//    for (i = 0; i < 4; ++i)
//        temp_points.emplace_back(vertices[i]);
//    if (temp_points[0].x > temp_points[1].x) {
//        temp_point = temp_points[0];
//        temp_points.erase(temp_points.begin());
//        temp_points.emplace_back(temp_point);
//    }
//    ellipse_Yaxis = getDistance(temp_points[0], temp_points[1]) / 2;
//    ellipse_Xaxis = getDistance(temp_points[1], temp_points[2]) / 2;
//    cout << "ellipse_Xaxis" << ellipse_Xaxis << endl;
//    cout << "ellipse_Yaxis" << ellipse_Yaxis << endl;
    if (isnan(ellipse_Xaxis) || isnan(ellipse_Yaxis))
        return false;
    else
        return true;
}

//返回极坐标下的角度,角度制
float RuneSolver::toPolarCoordinates(Point2f temp_point, Point2f origin_point) {
    float angle, rad;
    //x轴从左向右，y轴从下到上
    Point2f new_point = Point2f(temp_point.x - origin_point.x, temp_point.y - origin_point.y);
    rad = getDistance(temp_point, origin_point);
    if (new_point.y >= 0)
        angle = static_cast<float>(acos(new_point.x / rad) * 180 / PI);
    else
        angle = static_cast<float>(360 - acos(new_point.x / rad) * 180 / PI);
    return angle;
}

bool RuneSolver::solveDirection() {
    int i;
    if (angle_set.size() < QUEUE_SIZE)
        return false;
    angle_array.clear();
    findCenter();
    if (isFindCenter) {
        for (i = 0; i < angle_set.size(); ++i) {
            float angle = toPolarCoordinates(angle_set[i], ellipse_center);
            angle_array.emplace_back(angle);
        }
    }
    judgeDirection();
    if (direction == DIR_DEFAULT)
        return false;
    else
        return true;
}

void RuneSolver::judgeDirection() {
    double total = 0;
    int num = 0;
    double delta;
    for (int i = 1; i < angle_set.size(); i++) {
        delta = angle_array[i] - angle_array[i - 1];
        if (delta < -180)
            delta += 360;
        else if (delta > 180)
            delta -= 360;
        //发生目标切换
        if (abs(delta) > 30)
            continue;
        total += delta;
        num++;
    }
    double average = total / num;
    if (abs(average) < 0.1) {
        direction = DIR_STATIC;
    } else if (average > 0) {
        direction = DIR_CCW;
    } else {
        direction = DIR_CW;
    }
    cout << "旋转方向" << direction << endl;
//    for(int i = 0; i < angle_array.size(); i++){
//        cout << i << "号极坐标角度:" << angle_array[i] << endl;;
//    }
}

void RuneSolver::getangularvelocity() {
    if (angle_set.size() < QUEUE_SIZE)
        return;
    angle_array.clear();
    if (isFindCenter) {
        for (int i = 0; i < angle_set.size(); ++i) {
            float angle = toPolarCoordinates(angle_set[i], ellipse_center);
            angle_array.emplace_back(angle);
        }
    }
    int num = 0;
    double delta, temp_velocity;
    double delta_velocity = 0;
    for (int i = 1; i < angle_set.size(); i++) {
        delta = angle_array[i] - angle_array[i - 1];
        if (delta < -180)
            delta += 360;
        else if (delta > 180)
            delta -= 360;
        //发生目标切换
        if (abs(delta) > 30)
            break;
        temp_velocity = fabs(delta) / 0.040;
        delta_velocity += temp_velocity;
        num++;
    }
    if(direction == DIR_DEFAULT){
        angular_velocity = 0;
        return;
    }else if(direction == DIR_STATIC){
        angular_velocity = 0;
    }else{
        angular_velocity = delta_velocity / num;
    }
    if(isnan(angular_velocity)){
        angular_velocity = 0;
    }
}

void RuneSolver::predict() {
    if (mode == MODE_SMALL) {
        predict_angle = hit_angle;
        return;
    }
    Point2f prepoint = hit_angle - ellipse_center;
    Point2f previouspoint;
//    cout << "ellipse_angle" << ellipse_angle << endl;
//    cout << "ellipse_center:(" << ellipse_center.x << "," << ellipse_center.y << ")" << endl;
    previouspoint.x = static_cast<float>(prepoint.x * cos(ellipse_angle * PI / 180) +
                                         prepoint.y * sin(ellipse_angle * PI / 180));
    previouspoint.y = static_cast<float>(-prepoint.x * sin(ellipse_angle * PI / 180) +
                                         prepoint.y * cos(ellipse_angle * PI / 180));
//    cout << "previouspoint:(" << previouspoint.x << ", " << previouspoint.y << ")" << endl;
    float current_angle = toPolarCoordinates(previouspoint, Point2f(0.0f, 0.0f));
    float next_angle = 0;
    float dpitch, dyaw;

//    float angle_offset = 0;
//    angle_offset = static_cast<float>(angular_velocity * flight_time);
    switch (direction) {
        case DIR_DEFAULT:
            return;
            break;
        case DIR_CW:
            next_angle = current_angle - ANGLE_OFFSET;
//            next_angle = current_angle - angle_offset;
            break;
        case DIR_CCW:
            next_angle = current_angle + ANGLE_OFFSET;
//            next_angle = current_angle + angle_offset;
            break;
        case DIR_STATIC:
            next_angle = current_angle;
            break;
    }
    if (next_angle > 360) {
        next_angle -= 360;
    }
    if (next_angle < 0) {
        next_angle += 360;
    }
//    cout << "current_angle:" << current_angle << endl;
//    cout << "next_angle:" << next_angle << endl;
    next_angle = static_cast<float>(next_angle * PI / 180);
//    float radius = static_cast<float>(ellipse_Xaxis * ellipse_Yaxis /
//                   sqrt(ellipse_Yaxis * ellipse_Yaxis * cos(next_angle) * cos(next_angle) +
//                        ellipse_Xaxis * ellipse_Xaxis * sin(next_angle) * sin(next_angle)));
//    dpitch = static_cast<float>(radius * cos(next_angle));
//    dyaw = static_cast<float>(radius * sin(next_angle));
    dpitch = static_cast<float>(ellipse_Xaxis * cos(next_angle));
    dyaw = static_cast<float>(ellipse_Yaxis * sin(next_angle));
    //预测的打击角度
    Point2f predict_point;
    predict_point.x = static_cast<float>(dpitch * cos(ellipse_angle * PI / 180) - dyaw * sin(ellipse_angle * PI / 180));
    predict_point.y = static_cast<float>(dpitch * sin(ellipse_angle * PI / 180) + dyaw * cos(ellipse_angle * PI / 180));
    predict_angle = ellipse_center + predict_point;
//    cout << "prepoint:(" << prepoint.x << ", " << prepoint.y << ")" << endl;
//    cout << "predict_point:(" << predict_point.x << ", " << predict_point.y << ")" << endl;
//    cout << "hit_angle:(" << hit_angle.x << ", " << hit_angle.y << ")" << endl;
//    cout << "predict_angle:(" << predict_angle.x << ", " << predict_angle.y << ")" << endl;
}

void RuneSolver::draw(Mat &src, RotatedRect aim) {
    Point2f pt[4];
    aim.points(pt);
    for (int i = 1; i < 4; i++) {
        line(src, pt[i], pt[i - 1], Scalar(0, 255, 0), 1);
    }
    line(src, pt[3], pt[0], Scalar(0, 255, 0), 1);
}
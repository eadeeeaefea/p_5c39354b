/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Lu Zhan

 Detail:
 *****************************************************************************/

#include "dimension_runesolver.h"

void draw(Mat &src, RotatedRect &rotaterect) {
    Point2f pt[4];
    rotaterect.points(pt);
    line(src, pt[0], pt[1], Scalar(255, 0, 0), 2);
    line(src, pt[1], pt[2], Scalar(255, 0, 0), 2);
    line(src, pt[2], pt[3], Scalar(255, 0, 0), 2);
    line(src, pt[3], pt[0], Scalar(255, 0, 0), 2);
}

RuneSolver::RuneSolver() {
    isFindCenterR = false;
    isCalibrated = false;
    RuneSolver_centers.reserve(50);
    arrow_centers.reserve(50);
    mode = MODE_DEFAULT;
    object_Points.reserve(50);
}

RuneSolver::~RuneSolver() {
}

void RuneSolver::init(const FileStorage &file_storage) {
    file_storage["camera_matrix"] >> CAMERA_MATRIX;
    file_storage["distortion_coeff"] >> DISTCOEFFS;
}

void RuneSolver::run(const Mat &image, const int enemy_color, double &x, double &y, double &z, double ptz_pitch,
                     double ptz_yaw) {
    // Timer clock;
    image.copyTo(src);
    shoot = false;
    isLoseAllTargets = false;
    isFindRuneSolver = false;
    isFindArrow = false;
    preprocess(enemy_color);
    //找能量板 找箭头
    if (findRuneSolver() || findArrow()) {
        draw(src, target_RuneSolver);
        imshow("src", src);
    } else {
        cout << "丢失目标" << endl;
        x = y = z = 0;
        return;
    }

    //箭头区域和能量板是否匹配
    if (!match()) {
        cout << "不匹配\n" << endl;
        x = y = z = 0;
        return;
    }

    //求解相对坐标
    solveRealPostiton(target_RuneSolver);
    x = translation_matrix.at<double>(0, 0) / 1000;
    y = (translation_matrix.at<double>(1, 0) - 51.4469) / 1000;
    z = (translation_matrix.at<double>(2, 0) + 140.7033) / 1000;
    cout << "预测之前的坐标值:"  << "(" << x << ", " << y << ", " << z << ")" << endl;
    coordinate_transformation(x, y, z, ptz_pitch, ptz_yaw); //相对绝对的坐标Point3f
    cout << "预测之前的绝对坐标值" << "(" << x << "," << y << "," << z << ")" << endl;
    runecenter = Point3f((float) x, (float) y, (float) z);
//    cout << "runecenter:" << runecenter << endl;
    updateQueue();
//    cout << "runesolver_points.size: " << runesolver_points.size() << endl;
    if(runesolver_points.size() >= QUEUE_SIZE) {
        if (!fit()) {
            x = y = z = 0;
        }
    }
    x = result_point.at<float>(0, 0);
    y = result_point.at<float>(1, 0);
    z = result_point.at<float>(2, 0);
    anti_coordinate_transformation(x, y, z, ptz_pitch, ptz_yaw);
//    cout << "球心绝对坐标：(" << x0 << ", " << y0 << ", " << z0 << ")" << endl;
//    cout << "绝对平面拟合方程:"  << "z = " << A << " * x + " << B << " * y + " << C << endl;
//    cout << "预测后的坐标值:" << "(" << x << ", " << y << ", " << z << ")" << endl;
    shoot = true;
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

    element = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(temp_bin, temp_bin, element, Point(-1, -1), 3);
#ifdef SHOW_IMAGE
    imshow("复原后", temp_bin);
#endif
#else
    cv::cuda::GpuMat temp_bin_;
    cv::Mat temp_bin;
    temp_bin_.upload(bin);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
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
        if (area < MIN_RuneSolver_AREA || area > MAX_RuneSolver_AREA){
            cout << "hello0" << endl;
            continue;
        }

        target_contour = contours[i];
        if (target_contour.size() <= 5){
            cout << "hello1" << endl;
            continue;
        }
        temp_rect = fitEllipse(target_contour);
        temp_rect.size.height *= 0.75;
        temp_rect.size.width *= 0.75;
        //按宽高比筛选
        float width, height;
        width = max(temp_rect.size.width, temp_rect.size.height);
        height = min(temp_rect.size.width, temp_rect.size.height);
        ratio = width / height;
//        cout<<"能量板比例："<<ratio<<'\n';
        if (ratio < MIN_RuneSolver_RATIO || ratio > MAX_RuneSolver_RATIO){
            cout << "hello2" << endl;
            continue;
        }

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
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
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
    cout << "-------------------------------------\n";
    return isFindArrow;

}

//更新中心点队列数据
void RuneSolver::updateQueue() {
    if (runesolver_points.size() >= QUEUE_SIZE) {
        runesolver_points.erase(runesolver_points.begin());
    }
    runesolver_points.emplace_back(runecenter);
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
    dis = sqrt(pow(image_Points[0].x - image_Points[1].x, 2) + pow(image_Points[0].y - image_Points[1].y, 2));
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

void RuneSolver::coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {
    double absolute_x;
    double absolute_y;
    double absolute_z;
    absolute_x = x * cos(ptz_yaw) - y * sin(ptz_pitch) * sin(ptz_yaw) -
                 z * cos(ptz_pitch) * sin(ptz_yaw);
    absolute_y = y * cos(ptz_pitch) - z * sin(ptz_pitch);
    absolute_z = x * sin(ptz_yaw) + y * sin(ptz_pitch) * cos(ptz_yaw) +
                 z * cos(ptz_pitch) * cos(ptz_yaw);
    x = absolute_x;
    y = absolute_y;
    z = absolute_z;
}

void RuneSolver::anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw) {
    double objective_x;
    double objective_y;
    double objective_z;
    objective_x = x * cos(ptz_yaw) + z * sin(ptz_yaw);
    objective_y = -x * sin(ptz_yaw) * sin(ptz_pitch) + y * cos(ptz_pitch) +
                  z * cos(ptz_yaw) * sin(ptz_pitch);
    objective_z = -x * sin(ptz_yaw) * cos(ptz_pitch) - y * sin(ptz_pitch) +
                  z * cos(ptz_yaw) * cos(ptz_pitch);
    x = objective_x;
    y = objective_y;
    z = objective_z;
}

bool RuneSolver::fit() {
//    //平面拟合
//    cv::Mat MatrixcoefficientA = cv::Mat(3, 3, CV_32F, Scalar(0)); //矩阵系数A
//    cv::Mat MatrixcoefficientB = cv::Mat(3, 1, CV_32F, Scalar(0)); //矩阵系数B4
//    //球拟合
//    cv::Mat SphericalmatrixA = cv::Mat(3, 3, CV_32F, Scalar(0)); //矩阵系数A
//    cv::Mat SphericalmatrixB = cv::Mat(3, 1, CV_32F, Scalar(0)); //矩阵系数B
//    double meanx2 = 0, meanx = 0, meanx3 = 0;
//    double meany2 = 0, meany = 0, meany3 = 0;
//    double meanz2 = 0, meanz = 0, meanz3 = 0;
//    double meanxy = 0, meanxz = 0, meanyz = 0;
//    double meanx2y = 0, meanx2z = 0;
//    double meany2x = 0, meany2z = 0;
//    double meanz2x = 0, meanz2y = 0;
//    for (int i = 0; i < runesolver_points.size(); i++) {
//        //A
//        MatrixcoefficientA.at<float>(0, 0) = MatrixcoefficientA.at<float>(0, 0) + pow(runesolver_points[i].x, 2.0);
//        MatrixcoefficientA.at<float>(0, 1) =
//                MatrixcoefficientA.at<float>(0, 1) + runesolver_points[i].x * runesolver_points[i].y;
//        MatrixcoefficientA.at<float>(0, 2) = MatrixcoefficientA.at<float>(0, 2) + runesolver_points[i].x;
//        MatrixcoefficientA.at<float>(1, 0) =
//                MatrixcoefficientA.at<float>(1, 0) + runesolver_points[i].x * runesolver_points[i].y;
//        MatrixcoefficientA.at<float>(1, 1) = MatrixcoefficientA.at<float>(1, 1) + pow(runesolver_points[i].y, 2.0);
//        MatrixcoefficientA.at<float>(1, 2) = MatrixcoefficientA.at<float>(1, 2) + runesolver_points[i].y;
//        MatrixcoefficientA.at<float>(2, 0) = MatrixcoefficientA.at<float>(2, 0) + runesolver_points[i].x;
//        MatrixcoefficientA.at<float>(2, 1) = MatrixcoefficientA.at<float>(2, 1) + runesolver_points[i].y;
//        MatrixcoefficientA.at<float>(2, 2) = runesolver_points.size();
//        //B
//        MatrixcoefficientB.at<float>(0, 0) =
//                MatrixcoefficientB.at<float>(0, 0) + runesolver_points[i].x * runesolver_points[i].z;
//        MatrixcoefficientB.at<float>(1, 0) =
//                MatrixcoefficientB.at<float>(1, 0) + runesolver_points[i].y * runesolver_points[i].z;
//        MatrixcoefficientB.at<float>(2, 0) = MatrixcoefficientB.at<float>(2, 0) + runesolver_points[i].z;
//    }
//    //判断矩阵是否奇异
//    double flat_determ = determinant(MatrixcoefficientA);
//    if (abs(flat_determ) < 0.001) {
//        cout << "平面拟合矩阵系数A为奇异矩阵" << endl;
//        return false;
//    }
//    Mat flat_inv; // 系数矩阵MatrixcoefficientA的矩阵
//    invert(MatrixcoefficientA, flat_inv);
//    flat_result = flat_inv * MatrixcoefficientB; //平面拟合:第一个为a, 第二个为b, 第三个为c
//    A = flat_result.at<float>(0, 0);
//    B = flat_result.at<float>(1, 0);
//    C = flat_result.at<float>(2, 0);
//    cout << "flat_inv" << flat_inv << endl;
//    cout << "MatrixcoefficientB" << MatrixcoefficientB << endl;
//    cout << "flat_result" << flat_result << endl;
//    //球拟合
//    for (int i = 0; i < runesolver_points.size(); i++) {
//        meanx = meanx + runesolver_points[i].x;
//        meanx2 = meanx2 + runesolver_points[i].x * runesolver_points[i].x;
//        meanx3 = meanx3 + runesolver_points[i].x * runesolver_points[i].x * runesolver_points[i].x;
//        meany = meany + runesolver_points[i].y;
//        meany2 = meany2 + runesolver_points[i].y * runesolver_points[i].y;
//        meany3 = meany3 + runesolver_points[i].y * runesolver_points[i].y * runesolver_points[i].y;
//        meanz = meanz + runesolver_points[i].z;
//        meanz2 = meanz2 + runesolver_points[i].z * runesolver_points[i].z;
//        meanz3 = meanz3 + runesolver_points[i].z * runesolver_points[i].z * runesolver_points[i].z;
//        meanxy = meanxy + runesolver_points[i].x * runesolver_points[i].y;
//        meanxz = meanxz + runesolver_points[i].x * runesolver_points[i].z;
//        meanyz = meanyz + runesolver_points[i].y * runesolver_points[i].z;
//        meanx2y = meanx2y + runesolver_points[i].x * runesolver_points[i].x * runesolver_points[i].y;
//        meanx2z = meanx2z + runesolver_points[i].x * runesolver_points[i].x * runesolver_points[i].z;
//        meany2x = meany2x + runesolver_points[i].y * runesolver_points[i].y * runesolver_points[i].x;
//        meany2z = meany2z + runesolver_points[i].y * runesolver_points[i].y * runesolver_points[i].z;
//        meanz2x = meanz2x + runesolver_points[i].z * runesolver_points[i].z * runesolver_points[i].x;
//        meanz2y = meanz2y + runesolver_points[i].z * runesolver_points[i].z * runesolver_points[i].y;
//    }
//    meanx = meanx / runesolver_points.size();
//    meany = meany / runesolver_points.size();
//    meanz = meanz / runesolver_points.size();
//    meanx2 = meanx2 / runesolver_points.size();
//    meany2 = meany2 / runesolver_points.size();
//    meanz2 = meanz2 / runesolver_points.size();
//    meanx3 = meanx3 / runesolver_points.size();
//    meany3 = meany3 / runesolver_points.size();
//    meanz3 = meanz3 / runesolver_points.size();
//    meanxy = meanxy / runesolver_points.size();
//    meanyz = meanyz / runesolver_points.size();
//    meanxz = meanxz / runesolver_points.size();
//    meanx2y = meanx2y / runesolver_points.size();
//    meanx2z = meanx2z / runesolver_points.size();
//    meany2x = meany2x / runesolver_points.size();
//    meany2z = meany2z / runesolver_points.size();
//    meanz2x = meanz2x / runesolver_points.size();
//    meanz2y = meanz2y / runesolver_points.size();
//    SphericalmatrixA.at<float>(0, 0) = meanx2 - meanx * meanx;
//    SphericalmatrixA.at<float>(0, 1) = meanxy - meanx * meany;
//    SphericalmatrixA.at<float>(0, 2) = meanxz - meanx * meanz;
//    SphericalmatrixA.at<float>(1, 0) = meanxy - meanx * meany;
//    SphericalmatrixA.at<float>(1, 1) = meany2 - meany * meany;
//    SphericalmatrixA.at<float>(1, 2) = meanyz - meany * meanz;
//    SphericalmatrixA.at<float>(2, 0) = meanxz - meanx * meanz;
//    SphericalmatrixA.at<float>(2, 1) = meanyz - meany * meanz;
//    SphericalmatrixA.at<float>(2, 2) = meanz2 - meanz * meanz;
//    SphericalmatrixB.at<float>(0, 0) = meanx3 - meanx * meanx2 + meany2x - meanx * meany2 + meanz2x - meanx * meanz2;
//    SphericalmatrixB.at<float>(1, 0) = meanx2y - meanx2 * meany + meany3 - meany * meany2 + meanz2y - meany * meanz2;
//    SphericalmatrixB.at<float>(2, 0) = meanx2z - meanx2 * meanz + meany2z - meanz * meany2 + meanz3 - meanz * meanz2;
//    cout << "SphericalmatrixA" << SphericalmatrixA << endl;
//    cout << "SphericalmatrixB" << SphericalmatrixB << endl;
//    double spherical_determ = determinant(SphericalmatrixA);
//    if (abs(spherical_determ) < 0.001) {
//        cout << "球拟合矩阵系数A为奇异矩阵" << endl;
//        return false;
//    }
//    Mat inv_spherical;
//    invert(SphericalmatrixA, inv_spherical);
//    spherical_result = inv_spherical * SphericalmatrixB;
//    cout << "spherical_result" << spherical_result << endl;
//    x0 = spherical_result.at<float>(0, 0);
//    y0 = spherical_result.at<float>(1, 0);
//    z0 = spherical_result.at<float>(2, 0);
//    return true;
}

bool RuneSolver::predict() {

}

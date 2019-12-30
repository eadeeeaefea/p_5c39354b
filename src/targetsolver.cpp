/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#include "targetsolver.h"
#include <iostream>


TargetSolver::TargetSolver() {

}

TargetSolver::~TargetSolver() {

}

void TargetSolver::init(const FileStorage &file_storage) {
    file_storage["camera_matrix"] >> camera_matrix;
    file_storage["distortion_coeff"] >> distortion_coeff;
    solve_algorithm = SOLVEPNP_EPNP;
}

void TargetSolver::run(const RotatedRect &armor, Target &target) {
#ifdef RUNNING_TIME
    Timer timer;
    timer.start();
#endif

    if (armor.size.empty()) {
        target.x = 0;
        target.y = 0;
        target.z = 0;
        return;
    }

    bool is_big_armor = 0;
    double ratio = (armor.size.width > armor.size.height) ?
                   (armor.size.width / armor.size.height) :
                   (armor.size.height / armor.size.width);
    is_big_armor = ratio > 4.8;
    solvePnP4Points(armor, is_big_armor, rotate_mat, trans_mat);
    camera2ptzTransform(trans_mat, target);

#ifdef RUNNING_TIME
    cout << "solve target time: " << timer.getTime() << "ms" << endl;
    timer.stop();
#endif
}

void TargetSolver::solvePnP4Points(const RotatedRect &rect,
                                   const bool is_big_armor,
                                   Mat &rotate,
                                   Mat &trans) {
    static Point2f vertices[4];
    static Point2f left_up, left_down, right_up, right_down;
    rect.points(vertices);
    sort(vertices, vertices + 4, [](const Point2f &p1, const Point2f &p2) {
           return p1.x < p2.x;
       }
    );
    if (vertices[0].y < vertices[1].y) {
       left_up = vertices[0];
       left_down = vertices[1];
    } else {
       left_up = vertices[1];
       left_down = vertices[0];
    }
    if (vertices[2].y < vertices[3].y) {
       right_up = vertices[2];
       right_down = vertices[3];
    } else {
       right_up = vertices[3];
       right_down = vertices[2];
    }

    static vector<Point2f> points2d;
    points2d.clear();
    points2d.push_back(left_up);
    points2d.push_back(right_up);
    points2d.push_back(right_down);
    points2d.push_back(left_down);

    static double half_w, half_h;
    if (is_big_armor) {
        half_w = 115.04;  // big_armor_width = 230.08
        half_h = 27.74;   // big_armor_height = 55.48
    } else {
        half_w = 68.98;   // small_armor_width = 137.96
        half_h = 27.74;   // small_armor_height = 55.48
    }

    static vector<Point3f> points3d;  // points3d中的点需和points2d中的点按顺序一一对应
    points3d.clear();
    points3d.push_back(Point3f(-half_w, -half_h, 0));
    points3d.push_back(Point3f(half_w, -half_h, 0));
    points3d.push_back(Point3f(half_w, half_h, 0));
    points3d.push_back(Point3f(-half_w, half_h, 0));

    // calculating speed: P3P > EPNP > Iterative
    solvePnP(points3d, points2d, camera_matrix, distortion_coeff, rotate, trans, false, solve_algorithm);
    // Rodrigues(rotate_mat, rotate_mat);  // 使用旋转矩阵时需将注释去掉
}

void TargetSolver::camera2ptzTransform(const Mat &camera_position,
                                       Target &ptz_position) {
    ptz_position.x = camera_position.at<double>(0,0) / 1000;
    ptz_position.y = (camera_position.at<double>(1,0) - 51.4469) / 1000;  // 云台坐标系y向上
    ptz_position.z = (camera_position.at<double>(2,0) + 140.7033) / 1000;
}

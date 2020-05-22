/**
 * @file stereo.h
 * @author Bruce Hou
 * @email 402156782@qq.com
 * @license Copyright© HITwh HERO-RoboMaster2020 Group
 */
#include <opencv2/opencv.hpp>
#include "timer.h"

using namespace std;
using namespace cv;


class Stereo {
private:
    Timer timer;
    const float PI = 3.14159265358;

    float f; // f 归一化后的焦距，单位像素
    float fx, fy; // 当像素尺寸单元x，y相等时fx=fy=f;
    float ux,vy;
    float b; // 双目基线，两摄像头之间的距离，单位米

    Mat Rl, Rr, Pl, Pr, Q;

    Mat K_L, D_L, K_R, D_R, R_CR, T_CR;

    Mat lmap1, lmap2, rmap1, rmap2;


    //pt to object coordinates
    vector<Point3f> object;
    vector<Point2f> im_pt;
    Point3f objectpt; // 目标三维坐标点

    //SGBM
    int mindisparity = 0;
    int ndisparities = 64;
    int SADWindowSize = 15;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, ndisparities, SADWindowSize);

    //BM
    Ptr<StereoBM> bm = StereoBM::create(ndisparities, SADWindowSize);
public:
    Stereo();

    ~Stereo();

    void init(const FileStorage &file_storage);

    void pt2object(Point2f pt1, Point2f pt2, Point3f &objectpt); // pt1, pt2必须是在原图的像素坐标

    void sgbm_get(Mat leftimg, Mat rightimg, Mat &disp, Mat &xyz);

    void bm_get(Mat leftimg, Mat rightimg, Mat &disp);

    void correct_image(Mat &leftimg, Mat &rightimg);

    void get_xyz(Mat disp, Mat &xyz);


};
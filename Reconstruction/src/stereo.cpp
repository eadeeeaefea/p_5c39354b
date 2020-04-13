#include "stereo.h"

Stereo::Stereo() {

}

Stereo::~Stereo() {

}

void Stereo::init(const FileStorage &file_storage) {
    file_storage["LeftIntrinsicMatrix"] >> K_L;
    file_storage["RightIntrinsicMatrix"] >> K_R;
    file_storage["LeftRadialDistortion"] >> D_L;
    file_storage["RightRadialDistortion"] >> D_R;
    file_storage["RotationOfCameraR"] >> R_CR;
    file_storage["TranslationOfCameraR"] >> T_CR;
    stereoRectify(K_L, D_L, K_R, D_R, Size(640, 480), R_CR, T_CR, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY, -1,
                  Size(640, 480));

}

void Stereo::pt2object(Point2f pt1, Point2f pt2, Point3f &objectpt) {
    /**
    // 基于矩阵方法，参考 https://hitwhlc.yuque.com/hero-rm/fxhbfd/qwdnbv
    Mat G_one(2, 3, CV_32F);
    Mat G_two(2, 3, CV_32F);
    Mat invR_CR;
    Mat T_CRT;
    Mat temp1(2, 3, CV_32F);
    Mat temp2(2, 1, CV_32F);
    Mat A_one(4, 3, CV_32F);
    Mat invA_one(3, 4, CV_32F);
    Mat A_two(4, 1, CV_32F);
    Mat result(3, 1, CV_32F);

    invert(R_CR, invR_CR);
    //赋值
    G_one.at<float>(0, 0) = K_L.at<float>(0, 0);
    G_one.at<float>(0, 1) = 0;
    G_one.at<float>(0, 2) = K_L.at<float>(0, 2) - pt1.x;
    G_one.at<float>(1, 0) = 0;
    G_one.at<float>(1, 1) = K_L.at<float>(1, 1);
    G_one.at<float>(1, 2) = K_L.at<float>(1, 2) - pt1.y;

    temp1.at<float>(0, 0) = K_R.at<float>(0, 0);
    temp1.at<float>(0, 1) = 0;
    temp1.at<float>(0, 2) = K_R.at<float>(0, 2) - pt2.x;
    temp1.at<float>(1, 0) = 0;
    temp1.at<float>(1, 1) = K_R.at<float>(1, 1);
    temp1.at<float>(1, 2) = K_R.at<float>(1, 2) - pt2.y;

    G_two = temp1 * invR_CR;
    transpose(T_CR, T_CRT);
    temp2 = G_two * T_CRT;
    //矩阵拼接
    vconcat(G_one, G_two, A_one);
    invert(A_one, invA_one);
    A_two.at<float>(0, 0) = 0;
    A_two.at<float>(1, 0) = 0;
    A_two.at<float>(2, 0) = temp2.at<float>(0, 0);
    A_two.at<float>(3, 0) = temp2.at<float>(1, 0);

    // 求取三维坐标
    result = invA_one * A_two;

    objectpt.x = result.at<float>(0, 0);
    objectpt.y = result.at<float>(1, 0);
    objectpt.z = result.at<float>(2, 0);
    **/

    // 基于视差方法
    // 经过立体矫正，两者应该在一条基线上
    Mat disp(1, 1, CV_32F);
    Mat xyz;
    disp.at<float>(0, 0) = pt2.x - pt1.x;
    get_xyz(disp, xyz);
    objectpt.x = xyz.at<float>(1, 0);
    objectpt.y = xyz.at<float>(2, 0);
    objectpt.z = xyz.at<float>(3, 0);
}

void Stereo::sgbm_get(Mat leftimg, Mat rightimg, Mat &disp, Mat &xyz) {
    correct_image(leftimg, rightimg);
    int numberOfDisparities = ((leftimg.size().width / 8) + 15) & -16;
    int P1 = 8 * leftimg.channels() * SADWindowSize * SADWindowSize;
    int P2 = 32 * leftimg.channels() * SADWindowSize * SADWindowSize;
    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setPreFilterCap(15);
    sgbm->setUniquenessRatio(10);
//    sgbm->setBlockSize(SADWindowSize);
//    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
//    cout << numberOfDisparities << endl;
    sgbm->setSpeckleRange(2);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setDisp12MaxDiff(1);
//    sgbm->setMode(cv::StereoSGBM::MODE_HH);
    sgbm->compute(leftimg, rightimg, disp);
    get_xyz(disp, xyz);
//    disp.convertTo(disp, CV_32F, 1.0 / 16);//除以16得到真实视差值
//    normalize(disp, disp, 0, 255, NORM_MINMAX, CV_8UC1);
    disp.convertTo(disp, CV_8U, 255 / (numberOfDisparities * 16.0));
}

void Stereo::bm_get(Mat leftimg, Mat rightimg, Mat &disp) {
    correct_image(leftimg, rightimg);
    bm->setBlockSize(SADWindowSize);
    bm->setMinDisparity(mindisparity);
    bm->setNumDisparities(ndisparities);
    bm->setPreFilterSize(15);
    bm->setPreFilterCap(31);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(10);
    bm->setDisp12MaxDiff(1);
    copyMakeBorder(leftimg, leftimg, 0, 0, 80, 0, CV_HAL_BORDER_REPLICATE);  //防止黑边
    copyMakeBorder(rightimg, rightimg, 0, 0, 80, 0, CV_HAL_BORDER_REPLICATE);
    bm->compute(leftimg, rightimg, disp);
    disp.convertTo(disp, CV_32F, 1.0 / 16);                //除以16得到真实视差值
    normalize(disp, disp, 0, 255, NORM_MINMAX, CV_8UC1);
}

void Stereo::correct_image(Mat &leftimg, Mat &rightimg) {

    initUndistortRectifyMap(K_L, D_L, Mat(),
                            getOptimalNewCameraMatrix(K_L, D_L, leftimg.size(), 1,
                                                      leftimg.size(), 0), leftimg.size(), CV_16SC2, lmap1, lmap2);
    initUndistortRectifyMap(K_R, D_R, Mat(),
                            getOptimalNewCameraMatrix(K_R, D_R, rightimg.size(), 1,
                                                      rightimg.size(), 0), rightimg.size(), CV_16SC2, rmap1, rmap2);
    remap(leftimg, leftimg, lmap1, lmap2, INTER_LINEAR);
    remap(rightimg, rightimg, rmap1, rmap2, INTER_LINEAR);

}

void Stereo::get_xyz(Mat disp, Mat &xyz) {
    reprojectImageTo3D(disp, xyz, Q, true);
    xyz = xyz * 16;
}

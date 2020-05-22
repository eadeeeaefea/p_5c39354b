#include "kalman.h"

KF::KF() {

}

KF::~KF() {

}

void KF::run(float &x) {
    update_queue(x);
    if (target.size() < queue_size) {
        return;
    }
    if (!judgement()) {
        init_kf();
    } else {
        x = fobject(x);
    }
}

void KF::init_kf() {
    kf.init(2, 1, 0);
    kf.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);  //转移矩阵A[1,1;0,1]
    setIdentity(kf.measurementMatrix);                             //测量矩阵H
    setIdentity(kf.processNoiseCov, Scalar::all(1e-5));            //系统噪声方差矩阵Q
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));        //测量噪声方差矩阵R
    setIdentity(kf.errorCovPost, Scalar::all(1));                  //后验错误估计协方差矩阵P

    randn(kf.statePost, Scalar::all(0), Scalar::all(0.1));
}

void KF::update_queue(float x) {
    if (target.size() >= queue_size) {
        target.erase(target.begin());
    }
    target.emplace_back(x);
}

bool KF::judgement() {
    int count = 0;
    for (int i = 0; i < target.size(); i++) {
        if (fabs(target[i]) < 0.01) {
            count += 1;
        }
    }
    if (count > 0) {
        cout << "no filter" << endl;
        return false;
    }
    return true;
}

float KF::fobject(float x) {
    Mat predections = kf.predict();
    measurement.at<float>(0, 0) = x;
    measurement = kf.correct(measurement);
    return measurement.at<float>(0, 0);
}
#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "stereo.h"

using namespace cv;
using namespace std;

Mat xyz;

#define PARAM_PATH "../param/param.xml"

void onMouse(int event, int x, int y, int, void *) {
    Point origin;
    switch (event) {
        case EVENT_LBUTTONDBLCLK:
            origin = Point(x, y);
            cout << origin << "in world coordinate is :" << xyz.at<Vec3f>(origin) << endl;
            break;
    }
}

int main() {
    Stereo stereo;
    Timer timer;
    FileStorage file_storage(PARAM_PATH, FileStorage::READ);
    stereo.init(file_storage);
    VideoCapture cap(2);
    Mat oriframe, frame;
    Mat gray;
    Mat leftimg, rightimg;
    Mat result;
//    Mat xyz;

    int count = 0;
    while (true) {
        cap >> oriframe;
        if (oriframe.empty()) {
            break;
        }
        cvtColor(oriframe, oriframe, COLOR_BGR2GRAY);
        if (count == 2) {
            system(".././change.sh");
        }
        if (count != 3) {
            count++;
        }
        timer.start();
        resize(oriframe, frame, Size(1280, 480), (0, 0), (0, 0), INTER_AREA);
        leftimg = frame(Rect(0, 0, 640, 480));
        rightimg = frame(Rect(640, 0, 640, 480));
        stereo.sgbm_get(leftimg, rightimg, result, xyz);
//        cout << timer.getTime() << endl;
        timer.stop();
//        imshow("leftimg", leftimg);
//        cout << leftimg.size() << endl;
//        imshow("rightimg", rightimg);
        imshow("result", result);
        setMouseCallback("result", onMouse, 0);
        if (waitKey(1) >= 0) {
            break;
        }
    }
}


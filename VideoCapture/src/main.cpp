#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    Camera cam("/dev/video2", 640, 480, 0.1);

    while(true)
    {
        Mat src;
        cam.getImage(src);
        imshow("src", src);

        if(waitKey(1) == 27)
            break;    
    }

    return 0;
}
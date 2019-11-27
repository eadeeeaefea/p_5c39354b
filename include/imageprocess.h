#include<iostream>
#include <vector>
#include <cmath>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

class ImageProcess {
public:
    ImageProcess();

    ~ImageProcess();

    void find_red_areas(Mat InputArray, Mat OutputArray, double green_radio, double blue_radio);

    void find_blue_areas(Mat InputArray, Mat OutputArray, double red_radio, double green_radio);
};
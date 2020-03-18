#include <iostream>
#include "opencv2/opencv.hpp"
#include <cmath>

using namespace std;
using namespace cv;


void coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

void anti_coordinate_transformation(double &x, double &y, double &z, double ptz_pitch, double ptz_yaw);

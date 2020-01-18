#include "imageprocess.h"

using namespace std;
using namespace cv;

bool red_light_radio(double blue, double green, double red, double min_parameter, double max_parameter);

bool blue_light_radio(double blue, double green, double red, double min_parameter, double max_parameter);

bool green_light_radio(double blue, double green, double red, double min_parameter, double max_parameter);

ImageProcess::ImageProcess() {

};

ImageProcess::~ImageProcess() {

};

void ImageProcess::find_red_areas(Mat InputArray, Mat OutputArray, double green_radio, double blue_radio) {
    vector<Mat> channels;
    split(InputArray, channels);
    int rows = channels[0].rows;
    int cols = channels[0].cols;
    double r, g, b;
    for (int i = 0; i < rows; i++) {
        uchar *p0 = channels[0].ptr<uchar>(i);
        uchar *p1 = channels[1].ptr<uchar>(i);
        uchar *p2 = channels[2].ptr<uchar>(i);
        uchar *op = OutputArray.ptr<uchar>(i);
        for (int j = 0; j < cols; j++) {
            uchar &pix0 = *p0++;
            uchar &pix1 = *p1++;
            uchar &pix2 = *p2++;
            uchar &pixop = *op++;
            b = static_cast<double >(pix0);
            g = static_cast<double >(pix1);
            r = static_cast<double >(pix2);
            if (pix2 - pix0 > 100) {
                if (green_light_radio(b, g, r, 0.0001, green_radio) && blue_light_radio(b, g, r, 0.0001, blue_radio)) {
                    pixop = 255;
                } else {
                    pixop = 0;
                }
            } else {
                pixop = 0;
            }
        }
    }
}


void ImageProcess::find_blue_areas(Mat InputArray, Mat OutputArray, double red_radio, double green_radio) {
    vector<Mat> channels;
    split(InputArray, channels);
    int rows = channels[0].rows;
    int cols = channels[0].cols;
    double r, g, b;
    for (int i = 0; i < rows; i++) {
        uchar *p0 = channels[0].ptr<uchar>(i);
        uchar *p1 = channels[1].ptr<uchar>(i);
        uchar *p2 = channels[2].ptr<uchar>(i);
        uchar *op = OutputArray.ptr<uchar>(i);
        for (int j = 0; j < cols; j++) {
            uchar &pix0 = *p0++;
            uchar &pix1 = *p1++;
            uchar &pix2 = *p2++;
            uchar &pixop = *op++;
            b = static_cast<double >(pix0);
            g = static_cast<double >(pix1);
            r = static_cast<double >(pix2);
            if (pix0 - pix2 > 100) {
                if (red_light_radio(b, g, r, 0.0001, red_radio) && green_light_radio(b, g, r, 0.0001, green_radio)) {
                    pixop = 255;
                } else {
                    pixop = 0;
                }
            } else {
                pixop = 0;
            }
        }
    }
}


bool red_light_radio(double blue, double green, double red, double min_parameter, double max_parameter) {
    double radio = red / (red + green + blue);
    if (radio >= min_parameter && radio <= max_parameter) {
        return true;
    }
}

bool blue_light_radio(double blue, double green, double red, double min_parameter, double max_parameter) {
    double radio = blue / (red + green + blue);
    if (radio >= min_parameter && radio <= max_parameter) {
        return true;
    }
}

bool green_light_radio(double blue, double green, double red, double min_parameter, double max_parameter) {
    double radio = green / (red + green + blue);
    if (radio >= min_parameter && radio <= max_parameter) {
        return true;
    }
}

/**
 * @file dhcamera.h
 * @brief 
 * @details 
 * @author Li Haotian on 2020-03-10
 * @email lcyxlihaotian@126.com
 * @update Li Haotian on 2020-05-13
 * @version 
 * @license 2020 HITWH HERO-Robomaster Group
 */
#ifndef DHCAMERA_H
#define DHCAMERA_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <string>
#include <exception>

#include "GxIAPI.h"
#include "DxImageProc.h"

class DHCamera
{
private:
    GX_DEV_HANDLE m_hDevice = NULL; // device handle
    uchar *m_pBufferRGB;            // RGB image data, convert to cv::Mat
    int64_t m_nPayLoadSize;         // size of acquired raw image
    int64_t m_nPixelColorFilter;    // Bayer format

    bool is_open_;

public:
    DHCamera();
    ~DHCamera();

    //TODO: exposure delay
    void open(int64_t frame_width = 1280,
              int64_t frame_height = 1024,
              int64_t exposure_time = 1000, // unit: us  range:[20, 1e+06]
              double frame_speed = 210);    // max = 210

    void getImage(cv::Mat &image);
    cv::Mat getImage();

    bool isOpen();
    void close();
};

class DHCameraException : public std::exception
{
private:
    std::string e_what_;

public:
    DHCameraException() {}
    DHCameraException(const std::string &error) : e_what_(error) {}
    virtual ~DHCameraException() throw() {}
    virtual const char *what() const throw()
    {
        return e_what_.c_str();
    }
};

#endif // DHCAMERA_H
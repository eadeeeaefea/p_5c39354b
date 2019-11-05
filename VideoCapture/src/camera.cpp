#include "camera.h"

Camera::Camera(string dev_name,
               int frame_width,
               int frame_height,
               int exposure_time)
{
    camera_name_ = dev_name;
    cap_.open(camera_name_);
    if(!cap_.isOpened())
    {
        throw CameraException("Open camera failed.");
    }
    cap_.set(CAP_PROP_FRAME_WIDTH, frame_width);
    cap_.set(CAP_PROP_FRAME_HEIGHT, frame_height);
    cap_.set(CAP_PROP_EXPOSURE, exposure_time);
    cap_.set(CAP_PROP_FPS, 200);
    cap_.set(CAP_PROP_AUTO_EXPOSURE, 0.25); // manual exposure
}   

Camera::~Camera()
{
    if(cap_.isOpened())
    {
        cap_.release();
    }
}

void Camera::open(string dev_name,
          int frame_width,
          int frame_height,
          int exposure_time)
{
    if(cap_.isOpened())
    {
        return;
    }

    camera_name_ = dev_name;
    cap_.open(camera_name_);
    if(!cap_.isOpened())
    {
        throw CameraException("Open camera failed.");    
    }
    cap_.set(CAP_PROP_FRAME_WIDTH, frame_width);
    cap_.set(CAP_PROP_FRAME_HEIGHT, frame_height);
    cap_.set(CAP_PROP_EXPOSURE, exposure_time);    
}

bool Camera::isOpen()
{
    return cap_.isOpened();
}

void Camera::getImage(Mat &image)
{
    if(!cap_.isOpened())
    {
        throw CameraException("Get image failed, cam not opened.");
    }

    cap_ >> image;    

    if(!image.data)
    {
        throw CameraException("Read image failed.");
    }
}

Mat Camera::getImage()
{
    if(!cap_.isOpened())
    {
        throw CameraException("Get image failed, cam not opened.");
    }

    Mat src;
    cap_ >> src;
    if(!src.data)
    {
        throw CameraException("Read image failed.");
    }    
    
    return src;
}


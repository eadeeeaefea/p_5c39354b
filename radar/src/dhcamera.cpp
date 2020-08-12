/**
 * @file dhcamera.h
 * @brief 
 * @details 
 * @author Li Haotian on 2020-03-10
 * @email lcyxlihaotian@126.com
 * @update 
 * @version 
 * @license 2020 HITWH HERO-Robomaster Group
 */

#include "dhcamera.h"

DHCamera::DHCamera()
{
    is_open_ = false;
}

DHCamera::~DHCamera()
{
    if (m_pBufferRGB)
    {
        free(m_pBufferRGB);
        m_pBufferRGB = NULL;
    }
    if (m_hDevice)
    {
        GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
        GXCloseDevice(m_hDevice);
        GXCloseLib();
    }
}

void DHCamera::open(int64_t frame_width,
                    int64_t frame_height,
                    int64_t exposure_time,
                    double frame_speed)
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    GX_OPEN_PARAM openParam;
    uint32_t nDeviceNum = 0;
    openParam.accessMode = GX_ACCESS_EXCLUSIVE;
    openParam.openMode = GX_OPEN_INDEX;
    openParam.pszContent = "1";

    // initialize library
    emStatus = GXInitLib();
    if (emStatus != GX_STATUS_SUCCESS)
    {
        return;
    }
    // enumerate device
    emStatus = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((emStatus != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        return;
    }
    // open device
    emStatus = GXOpenDevice(&openParam, &m_hDevice);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        return;
    }

    // set continuous capture
    GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GXSetEnum(m_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    // set image width, height, exposure time...
    GXSetInt(m_hDevice, GX_INT_WIDTH, frame_width);
    GXSetInt(m_hDevice, GX_INT_HEIGHT, frame_height);
    GXSetFloat(m_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, frame_speed);
    // manual exposure time
    GXSetEnum(m_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_time);

    // get image size
    emStatus = GXGetInt(m_hDevice, GX_INT_PAYLOAD_SIZE, &m_nPayLoadSize);
    if ((emStatus != GX_STATUS_SUCCESS) || (m_nPayLoadSize <= 0))
    {
        return;
    }

    // judge whether camera supports bayer format
    bool m_bColorFilter;
    emStatus = GXIsImplemented(m_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &m_bColorFilter);

    if (m_bColorFilter)
    {
        GXGetEnum(m_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &m_nPixelColorFilter);
    }

    m_pBufferRGB = (uchar *)malloc((size_t)(frame_width * frame_height * 3));
    if (!m_pBufferRGB)
    {
        return;
    }

    emStatus = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        return;
    }

    is_open_ = true;
}

void DHCamera::getImage(cv::Mat &image)
{
    if (!is_open_)
    {
        return;
    }

    // GXGetImage input param
    GX_FRAME_DATA frame_data;
    // apply buffer according to m_nPayLoadSize
    frame_data.pImgBuf = (uchar *)malloc((size_t)m_nPayLoadSize);
    if (!frame_data.pImgBuf)
    {
        return;
    }

    // acquire one image frame
    while (GXGetImage(m_hDevice, &frame_data, 100) != GX_STATUS_SUCCESS)
    {
        sleep(1);
    }
    if (frame_data.nStatus == GX_FRAME_STATUS_SUCCESS)
    {
        DxRaw8toRGB24(frame_data.pImgBuf,
                      m_pBufferRGB,
                      (VxUint32)(frame_data.nWidth),
                      (VxUint32)(frame_data.nHeight),
                      RAW2RGB_NEIGHBOUR,
                      DX_PIXEL_COLOR_FILTER(m_nPixelColorFilter),
                      false);
        image = cv::Mat(cv::Size(frame_data.nWidth, frame_data.nHeight), CV_8UC3, m_pBufferRGB);

        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        std::swap(channels[0], channels[2]);
        cv::merge(channels, image);

        free(frame_data.pImgBuf);
    }
}

cv::Mat DHCamera::getImage()
{
    if (!is_open_)
    {
        return cv::Mat();
    }

    cv::Mat image;
    // GXGetImage input param
    GX_FRAME_DATA frame_data;
    // apply buffer according to m_nPayLoadSize
    frame_data.pImgBuf = (uchar *)malloc((size_t)m_nPayLoadSize);
    if (!frame_data.pImgBuf)
    {
        return cv::Mat();
    }

    // acquire one image frame
    while (GXGetImage(m_hDevice, &frame_data, 100) != GX_STATUS_SUCCESS)
    {
        sleep(1);
    }
    if (frame_data.nStatus == GX_FRAME_STATUS_SUCCESS)
    {
        DxRaw8toRGB24(frame_data.pImgBuf,
                      m_pBufferRGB,
                      (VxUint32)(frame_data.nWidth),
                      (VxUint32)(frame_data.nHeight),
                      RAW2RGB_NEIGHBOUR,
                      DX_PIXEL_COLOR_FILTER(m_nPixelColorFilter),
                      false);
        image = cv::Mat(cv::Size(frame_data.nWidth, frame_data.nHeight), CV_8UC3, m_pBufferRGB);
        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        std::swap(channels[0], channels[2]);
        cv::merge(channels, image);

        free(frame_data.pImgBuf);
    }

    return image;
}

bool DHCamera::isOpen()
{
    return is_open_;
}

void DHCamera::close()
{
    if (m_pBufferRGB)
    {
        delete[] m_pBufferRGB;
        m_pBufferRGB = NULL;
    }
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    emStatus = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
    emStatus = GXCloseDevice(m_hDevice);
    emStatus = GXCloseLib();
    m_hDevice = NULL;

    is_open_ = false;
}
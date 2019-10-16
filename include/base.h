/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_BASE_H
#define HERORM2020_BASE_H


#define RUNNING_TIME
#define SHOW_IMAGE
// #define TRACKBAR  // 使用时需取消SHOW_IMAGE的注释

// #define COMPILE_WITH_CUDA
// #define DISTORTION_CORRECT  // preprocess

#define ROI_ENABLE

// #define CAMERA_ONLY    // 调试时使用，不打开串口
#define ENEMY_COLOR 0    // 调试时使用，比赛时必须将其注释
#define ARMOR_ONLY
// #define RUNE_ONLY
// #define TEST 1    // 1-image, 2-video
#define IMAGE_PATH "../1.jpg"
#define VIDEO_PATH "1.avi"
// #define SAVE_VIDEO
// #define SAVE_VIDEO_ONLY    // without processing image
#define VIDEO_SAVED_PATH "1.avi"

#define PI 3.141592654

#define FRAME_WIDTH   640
#define FRAME_HEIGHT  480
#define EXPOSURE_TIME 400
// #define UART_NAME "/dev/ttyUSB0"
#define PARAM_PATH "../param/param.xml"


#endif  // HERORM2020_BASE_H

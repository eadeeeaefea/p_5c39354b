/**
 * @file base.h
 * @brief
 * @details 
 * @author Wang Xiaoyan on 2019.9.20
 * @email lcyxlihaotian@126.com
 * @update Li Haotian on 2020-05-13
 * @version 
 * @license 2020 HITWH HERO-Robomaster Group
 */

#ifndef HERORM2020_BASE_H
#define HERORM2020_BASE_H

#define RUNNING_TIME
#define SHOW_IMAGE
//#define TRACKBAR    // 使用时需取消SHOW_IMAGE的注释
//#define COMPILE_WITH_CUDA
//#define DISTORTION_CORRECT

// #define USE_CAMERA
// #define USE_SERIAL
// #define PLOT_DATA     // 必须开启 USE_SERIAL
// #define USE_MISSILE   // 使用飞镖识别程序
#define USE_ROBOT     // 使用机器人识别程序
#define ENEMY_COLOR 0 // 调试时使用，比赛时必须将其注释 0为红色 1为蓝色

//#define BGR           // 不可与HSV同时使用
#define HSV // 不可与BGR同时使用
//#define ROI_ENABLE    //雷达站代码中使用ROI可能出问题，先不用

#define TEST 2 // 1-image, 2-video
#define IMAGE_READ_PATH ""
#define ROBOT_VIDEO_READ_PATH "../material/RED_5_3.avi"
#define MISSILE_VIDEO_READ_PATH ""

/* 三种色彩判定方法，运行时只能选其中一种 */
#define CALCULATE_POINTS
//#define CALCULATE_INTENSITY
//#define CALCULATE_RATIO

// #define SAVE_VIDEO 1 // 1-save and run others. 2-save only

/* rarely changed constant variables */
#define ROBOT_FRAME_WIDTH 640
#define ROBOT_FRAME_HEIGHT 480
#define ROBOT_EXPOSURE_TIME 400
#define MISSILE_FRAME_WIDTH 640
#define MISSILE_FRAME_HEIGHT 640
#define MISSILE_EXPOSURE_TIME 400
#define PI 3.141592654
#define ROBOT_VIDEO_SAVE_PATH ""
#define MISSILE_VIDEO_SAVE_PATH ""
#define PARAM_PATH "../param/param1.xml" // 开自启时需改为绝对路径

#endif // HERORM2020_BASE_H

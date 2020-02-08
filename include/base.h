/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_BASE_H
#define HERORM2020_BASE_H


//#define RUNNING_TIME
//#define SHOW_IMAGE
//#define TRACKBAR    // 使用时需取消SHOW_IMAGE的注释

//#define COMPILE_WITH_CUDA
//#define DISTORTION_CORRECT
#define BGR    // 不可与HSV同时使用
//#define HSV    // 不可与BGR同时使用
#define ROI_ENABLE

//#define USE_CAMERA 1    // use camera enable and camera id
#define TEST 2    // 1-image, 2-video
//#define IMAGE_PATH "1.jpg"
#define VIDEO_PATH "/home/stayalive/Documents/HERO/2.avi" //1.avi 装甲板 2.avi rune
//#define SAVE_VIDEO 2    // 1-save video and run other programs, 2-save video only
//#define VIDEO_SAVED_PATH "1.avi"

#define ENEMY_COLOR 0   // 调试时使用，比赛时必须将其注释
//#define ARMOR_ONLY
#define RUNE_ONLY

//#define USE_CAN 0
//#define USE_SERIAL
//#define PLOT_DATA

#define PI 3.141592654

#define FRAME_WIDTH   640
#define FRAME_HEIGHT  480
#define EXPOSURE_TIME 500
#define PARAM_PATH "../param/param.xml"    // 开自启时需改为绝对路径

typedef struct {
    int mode;
    double yaw;
    double pitch;
} SendPack;

typedef struct {
    int enemy_color; //0-red, 1-blue
    int mode;
    double pitch;
    double yaw;
    double bullet_speed;
} ReadPack;

typedef struct {
    int plot_type;
    int curve_num;
    double plot_value[3];
} PlotPack;

enum Mode {
    ARMOR,
    RUNE
};
#endif  // HERORM2020_BASE_H

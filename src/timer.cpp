/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Zhou Yuxin on 2018.10.10

 Detail: 通过C内置的计时函数实现一个简易的计时器，确定程序中某一模块的运行时间。
 *****************************************************************************/

#include "timer.h"
#include <stdio.h>


Timer::Timer() {
    time_start = 0.0;
    time_end = 0.0;
    is_open = false;
}

Timer::~Timer() {

}

void Timer::start() {
    time_start = clock();
    is_open = true;
}

double Timer::restart() {
    double temp_time = getTime();
    time_start = clock();
    return temp_time;
}

void Timer::stop() {
    time_start = 0.0;
    is_open = false;
}

double Timer::getTime() {
    if (!is_open) {
        printf("Get time failed. Timer is not opened.\n");
        return 0.0;
    }

    double delta_time;
    time_end = clock();
    delta_time = static_cast<double>(time_end - time_start) * 1000.0 / CLOCKS_PER_SEC;
    return delta_time;
}

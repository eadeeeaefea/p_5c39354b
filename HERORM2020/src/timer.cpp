/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Zhou Yuxin on 2018.10.10

 Detail: 通过C内置的计时函数实现一个简易的计时器，确定程序中某一模块的运行时间。
 *****************************************************************************/

#include "timer.h"
#include <stdio.h>


Timer::Timer() {
    start_ = 0.0;
    end_ = 0.0;
    is_open_ = false;
}

Timer::~Timer() {

}

void Timer::start() {
    start_ = clock();
    is_open_ = true;
}

double Timer::restart() {
    double temp_time = getTime();
    start_ = clock();
    return temp_time;
}

void Timer::stop() {
    start_ = 0.0;
    is_open_ = false;
}

double Timer::getTime() {
    if (!is_open_) {
        printf("Get time failed. Timer is not opened.\n");
        return 0.0;
    }

    double delta_time;
    end_ = clock();
    delta_time = static_cast<double>(end_ - start_) * 1000.0 / CLOCKS_PER_SEC;
    return delta_time;
}

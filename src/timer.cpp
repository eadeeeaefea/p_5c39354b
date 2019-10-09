/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Zhou Yuxin on 2018.10.10

 Detail: 通过C内置的计时函数实现一个简易的计时器，确定程序中某一模块的运行时间。
 *****************************************************************************/

#include "timer.h"
#include <assert.h>


Timer::Timer() {
    time_start_ = 0.0;
    time_end_ = 0.0;
    status_ = STANDBY;
}

Timer::~Timer() {

}

void Timer::start() {
    time_start_ = clock();
    status_ = RUNNING;
}

double Timer::restart() {
    double temp_time = getTime();
    time_start_ = clock();
    return temp_time;
}

void Timer::stop() {
    time_start_ = 0.0;
    status_ = STANDBY;
}

double Timer::getTime() {

    assert(status_ == RUNNING);

    double delta_time;
    time_end_ = clock();
    delta_time = static_cast<double>(time_end_ - time_start_) * 1000.0 / CLOCKS_PER_SEC;
    return delta_time;
}

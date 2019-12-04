//
// Created by luzhan on 19-9-28.
//

#include "timer.hpp"
using namespace std;
using namespace cv;

Timer::Timer() {
    frequency = getTickFrequency();
    begin = getTickCount();
}

Timer::~Timer() {
    stop();
}

double Timer::stop() {
    end = getTickCount();
    cout<<"此程序段运行时间为： "<<(end - begin) / frequency * 1000<<" 毫秒\n";
    return  (end - begin) / frequency * 1000;
}

void Timer::stop(std::string str) {
    cout<<str;
    stop();
}
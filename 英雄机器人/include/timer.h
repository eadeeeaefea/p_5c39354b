/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Lu Zhan on 2019.12.15

 Detail:
 *****************************************************************************/

#ifndef HERORM2020_HERO_TIMER_H
#define HERORM2020_HERO_TIMER_H
#include <ctime>
#include <iostream>

class Timer
{
private:
    clock_t begin;
public:
    Timer()
    {
        begin = clock();
    }

    ~Timer()
    {
        stop();
    }

    void restart()
    {
        begin = clock();
    }

    void stop()
    {
        clock_t end = clock();
        std::cout<<"该程序段运行时间:"<< static_cast<double>(end - begin) * 1000.0 / CLOCKS_PER_SEC<<"ms\n";
    }

    void stop(const std::string &note)
    {
        clock_t end = clock();
        std::cout<<note<<"时间:"<< static_cast<double>(end - begin) * 1000.0 / CLOCKS_PER_SEC<<"ms \n";
    }
};
#endif //HERORM2020_HERO_TIMER_H




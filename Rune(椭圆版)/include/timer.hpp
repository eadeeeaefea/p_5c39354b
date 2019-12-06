//
// Created by luzhan on 19-9-28.
//

#ifndef ENERGY_FINDER_TIMER_HPP
#define ENERGY_FINDER_TIMER_HPP
#include <opencv2/opencv.hpp>

class Timer{
private:
    double begin;
    double end;
    double frequency;

public:
    Timer();
    ~Timer();
    double stop();
    void stop(std::string str);
};

#endif //ENERGY_FINDER_TIMER_HPP

#include "include/hero_plot.h"

HeroPlot::HeroPlot()
{
    setupVector();
}


void HeroPlot::run()
{
    std::thread th1(&HeroPlot::threadCalled, this);
    th1.join();
}

void HeroPlot::setupVector()
{
    for(int i = 0; i < 5; ++i)
    {
        pub_value_vec.append(0);
    }
}

void HeroPlot::addPoint(double value, HeroPlot::PLOT_TYPE type)
{
    static QTime time(QTime::currentTime());
    double time_from_start = time.elapsed()/1000.0; // time elapsed since start, in seconds

    pub_time_key = time_from_start;
    pub_value_vec[type] = value;

    emit addPointSignal(type);
}

void HeroPlot::threadCalled()
{
    while(true)
    {
        sleep(0.0001);
        addPoint(sin(pub_time_key), HeroPlot::PLOT_TYPE::TYPE_PITCH);
    }
}

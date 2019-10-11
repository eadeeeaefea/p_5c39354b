#ifndef HERO_PLOT_H
#define HERO_PLOT_H

#include <QThread>
#include <QVector>
#include <QMutex>
#include <QTimer>
#include <QTime>
#include <thread>
#include <opencv2/opencv.hpp>

class HeroPlot : public QThread
{
    Q_OBJECT

public:
    enum PLOT_TYPE
    {
        TYPE_X = 0,
        TYPE_Y = 1,
        TYPE_Z = 2,
        TYPE_YAW = 3,
        TYPE_PITCH = 4
    };

    // 0-4: x, y, z, yaw, pitch
    QVector<double> pub_value_vec;
    double pub_time_key = 0;

    // loop timer
    QTimer timer;

private:
    // locker
    QMutex mutex_;

    // plot variables
    double start_time_stamp_;

public:
    HeroPlot();
    void run();

private:
    void setupVector();
    void addPoint(double value, HeroPlot::PLOT_TYPE type);
    void threadCalled();

signals:
    void addPointSignal(int);

};

#endif // HERO_PLOT_H

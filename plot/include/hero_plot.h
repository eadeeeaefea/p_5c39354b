#ifndef HERO_PLOT_H
#define HERO_PLOT_H

#include <iostream>
#include <QThread>
#include <QVector>
#include <QTimer>
#include <QTime>
#include <QtMath>
#include <string>
#include "serialport.h"

class HeroPlot : public QThread
{
    Q_OBJECT

public:
    // 0-4: x, y, z, yaw, pitch
    QVector<double> pub_value_vec;
    double pub_time_key = 0;

    // loop timer
    QTimer timer;

private:
    // plot variables
    double start_time_stamp_;

    SerialPort::ReadPack read_pack_;
    SerialPort serialport;

public:
    HeroPlot(const char* serial_name);
    void run();

private:
    void setupParam();
    void addPoint(double value, int type);

signals:
    void addPointSignal(int);

};

#endif // HERO_PLOT_H

/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group
 Author: Wang Xiaoyan on 2019.10.12
 Detail: 串口相关。实现串口相关参数的设定和初始化，并以协调好的通信协议进行数据的收发。
 *****************************************************************************/

#ifndef HERORM2020_SERIALPORT_H
#define HERORM2020_SERIALPORT_H

#include <stdint.h>
#include <string>
#include <exception>

#include "base.h"

using std::string;
using std::exception;


class SerialPort {
private:
    string port_name_;
    long baud_rate_;
    int byte_size_;
    int parity_;
    int stop_bit_;
    int flow_control_;

    int fd_;
    bool is_open_;

public:
    SerialPort();
    SerialPort(const string &port_name,
               long baud_rate = 115200,
               int byte_size = 8,        // 5: 5bits, 6: 6bits, 7: 7bits, 8: 8bits
               int parity = 0,           // 0: none, 1: odd, 2: even
               int stop_bit = 1,         // 1: 1bit, 2: 2bits, 3: 1.5bits
               int flow_control = 0);    // 0: none, 1: software, 2: hardware

    ~SerialPort();

    void open();
    void open(const string &port_name,
              long baud_rate = 115200,
              int byte_size = 8,        // 5: 5bits, 6: 6bits, 7: 7bits, 8: 8bits
              int parity = 0,           // 0: none, 1: odd, 2: even
              int stop_bit = 1,         // 1: 1bit, 2: 2bits, 3: 1.5bits
              int flow_control = 0);    // 0: none, 1: software, 2: hardware
    bool isOpen();
    void close();

    void setBaudRate(long baud_rate);
    long getBaudRate();

    void setByteSize(int byte_size);
    int getByteSize();

    void setParity(int parity);
    int getParity();

    void setStopBit(int stop_bit);
    int getStopBit();

    void setFlowControl(int flow_control);
    int getFlowControl();

    void sendData(const SendPack &send_pack);
    bool readData(ReadPack &read_pack);

    bool sendPlot(const PlotPack &plot_pack);

private:
    void reconfigurePort();

};

class SerialException : public exception {
private:
    string e_what_;

public:
    SerialException() {}
    SerialException(const string &error) : e_what_(error) {}
    virtual ~SerialException() throw() {}
    virtual const char *what() const throw() {
        return e_what_.c_str();
    }

};


#endif // HERORM2020_SERIALPORT_H
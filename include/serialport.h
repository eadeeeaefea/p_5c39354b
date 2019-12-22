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

using std::string;
using std::exception;


typedef struct SendPack_t {
    int mode;
    double yaw;
    double pitch;
}SendPack;

typedef struct ReadPack_t {
    int enemy_color;  // 0-red, 1-blue
    int mode;
    double pitch;
    double yaw;
}ReadPack;

class SerialPort {
private:
    string port_name;
    long baud_rate;
    int byte_size;
    int parity;
    int stop_bit;
    int flow_control;

    int fd;
    bool is_open;

public:
    SerialPort();
    SerialPort(const string &_port_name,
               long _baud_rate = 115200,
               int _byte_size = 8,        // 5: 5bits, 6: 6bits, 7: 7bits, 8: 8bits
               int _parity = 0,           // 0: none, 1: odd, 2: even
               int _stop_bit = 1,         // 1: 1bit, 2: 2bits, 3: 1.5bits
               int _flow_control = 0);    // 0: none, 1: software, 2: hardware

    ~SerialPort();

    void open();
    void open(const string &_port_name,
              long _baud_rate = 115200,
              int _byte_size = 8,        // 5: 5bits, 6: 6bits, 7: 7bits, 8: 8bits
              int _parity = 0,           // 0: none, 1: odd, 2: even
              int _stop_bit = 1,         // 1: 1bit, 2: 2bits, 3: 1.5bits
              int _flow_control = 0);    // 0: none, 1: software, 2: hardware
    bool isOpen();
    void close();

    void set_baud_rate(long _baud_rate);
    long get_baud_rate();

    void set_byte_size(int _byte_size);
    int get_byte_size();

    void set_parity(int _parity);
    int get_parity();

    void set_stop_bit(int _stop_bit);
    int get_stop_bit();

    void set_flow_control(int _flow_control);
    int get_flow_control();

    void sendData(const SendPack &send_pack);
    bool readData(ReadPack &read_pack);

private:
    void reconfigurePort();

};

class SerialException : public exception {
private:
    string e_what;

public:
    SerialException() {}
    SerialException(const string &error) : e_what(error) {}
    virtual ~SerialException() throw() {}
    virtual const char *what() const throw() {
        return e_what.c_str();
    }

};


#endif // HERORM2020_SERIALPORT_H

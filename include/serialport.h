/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.3.12

 Detail: 串口相关。实现串口相关参数的设定和初始化，并以协调好的通信协议进行数据的收发。
 *****************************************************************************/

#ifndef HERORM2020_SERIALPORT_H
#define HERORM2020_SERIALPORT_H

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


class SerialPort {
private:
    speed_t baud_;  // 波特率
    int fd_port_;   // 串口的设备描述符

public:
    /**
     * @breif: 构造函数
     * @param: None
     * @return: None
     */
    SerialPort();

    /**
     * @breif: 析构函数
     * @param: None
     * @return: None
     */
    ~SerialPort();

    /**
     * @breif: 进行串口相关参数的设定和初始化
     * @param: port_name 串口的设备号
     *         baud_rate 波特率
     * @return: None
     */
    void init(const char *port_name, int baud_rate = 115200);

    /**
     * @breif: 发送数据。包括数据的打包
     * @param:
     * @return: 成功或失败
     */
    bool sendData(double yaw, double pitch);

    /**
     * @breif: 读取数据。包括数据的解包
     * @param:
     * @return: 成功或失败
     */
    bool readData(int &enemy_color, int &mode);

private:
    /**
    * @breif: 是否为正确数据包的判断函数
    * @param: buf 读取到的数据
    * @return: 是or否
    */
    bool isReadPackage(uint8_t *buf);

    /**
     * @breif: 打开串口，并设置为读写
     * @param: port_name 串口的设备号
     * @return: 通过read函数打开串口后的设备描述符
     */
    int openPort(const char *port_name);

    /**
     * @breif: 使能串口，并设置基本参数
     * @param: fd 串口的设备描述符
     * @return: 串口的设备描述符
     */
    int configurePort(int fd);

    /**
     * @breif: 关闭串口
     * @param: fd 串口的设备描述符
     * @return: None
     */
    void closePort(int fd);

};


#endif // HERORM2020_SERIALPORT_H

/******************************************************************************
 Copyright© HITwh HERO-RoboMaster2020 Group

 Author: Wang Xiaoyan on 2019.10.12

 Detail: 实现串口相关参数的设定和初始化，并通过协调好的通信协议进行数据的打包和解包，完成数据
         的收发。
 *****************************************************************************/

#include "serialport.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <errno.h>
#include <termios.h>

using std::string;


SerialPort::SerialPort() {
    fd = -1;
    is_open = false;
}

SerialPort::SerialPort(const string &_port_name,
                       long _baud_rate,
                       int _byte_size,
                       int _parity,
                       int _stop_bit,
                       int _flow_control) {
    port_name = _port_name;
    baud_rate = _baud_rate;
    byte_size = _byte_size;
    parity = _parity;
    stop_bit = _stop_bit;
    flow_control = _flow_control;
    fd = -1;
    is_open = false;
}

SerialPort::~SerialPort() {
   close();
}

void SerialPort::open() {
    if (port_name.empty()) {
        throw SerialException("Open port failed. Port name is empty.");
    }

    if (is_open) {
        throw SerialException("Open port failed. Port is already opened.");
    }

    fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd == -1) {
        throw SerialException("Open port failed. Bad file description.");
    }

    reconfigurePort();
    is_open = true;
}

void SerialPort::open(const string &_port_name,
                      long _baud_rate,
                      int _byte_size,
                      int _parity,
                      int _stop_bit,
                      int _flow_control) {
    port_name = _port_name;
    baud_rate = _baud_rate;
    byte_size = _byte_size;
    parity = _parity;
    stop_bit = _stop_bit;
    flow_control = _flow_control;

    if (port_name.empty()) {
        throw SerialException("Open port failed. Port name is empty.");
    }

    if (is_open) {
        throw SerialException("Open port failed. Port is already opened.");
    }

    fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd == -1) {
        throw SerialException("Open port failed. Bad file description.");
    }

    reconfigurePort();
    is_open = true;
}

bool SerialPort::isOpen() {
    return is_open;
}

void SerialPort::close() {
    if (is_open) {
        if (fd < 0) {
            throw SerialException("Close port failed. Bad file(open) description.");
        } else {
            int ret = ::close(fd);
            if (ret == 0) {
                fd = -1;
                is_open = false;
            } else {
                throw SerialException("Close port failed. Bad file(close) description.");
            }
        }
    } else {
        throw SerialException("Close port failed. Port is not opened.");
    }
}

void SerialPort::set_baud_rate(long _baud_rate) {
    if (is_open) {
        baud_rate = _baud_rate;
        reconfigurePort();
    } else {
        throw SerialException("Set baud rate failed. Port is not opened.");
    }
}

long SerialPort::get_baud_rate() {
    return baud_rate;
}

void SerialPort::set_byte_size(int _byte_size) {
    if (is_open) {
        byte_size = _byte_size;
        reconfigurePort();
    } else {
        throw SerialException("Set byte size failed. Port is not opened.");
    }
}

int SerialPort::get_byte_size() {
    return byte_size;
}

void SerialPort::set_parity(int _parity) {
    if (is_open) {
        parity = _parity;
        reconfigurePort();
    } else {
        throw SerialException("Set parity failed. Port is not opened.");
    }
}

int SerialPort::get_parity() {
    return parity;
}

void SerialPort::set_stop_bit(int _stop_bit) {
    if (is_open) {
        stop_bit = _stop_bit;
        reconfigurePort();
    } else {
        throw SerialException("Set stop bit failed. Port is not opened.");
    }
}

int SerialPort::get_stop_bit() {
    return stop_bit;
}

void SerialPort::set_flow_control(int _flow_control) {
    if (is_open) {
        flow_control = _flow_control;
        reconfigurePort();
    } else {
        throw SerialException("Set flow control failed. Port is not opened.");
    }
}

int SerialPort::get_flow_control() {
    return flow_control;
}

void SerialPort::reconfigurePort() {
    if (fd == -1) {
        throw SerialException("Configure port failed. Bad file description.");
    }

    struct termios options;

    if (::tcgetattr(fd, &options) < 0) {
        throw SerialException("tcgetattr error.");
    }

    options.c_cflag |= (tcflag_t) (CLOCAL | CREAD);
    options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG);
    options.c_oflag &= (tcflag_t) ~(OPOST);
    options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK | ISTRIP | INPCK);

    // setup baud rate
    speed_t baud;
    switch (baud_rate) {
        case 0:       baud = B0;       break;
        case 50:      baud = B50;      break;
        case 75:      baud = B75;      break;
        case 110:     baud = B110;     break;
        case 134:     baud = B134;     break;
        case 150:     baud = B150;     break;
        case 200:     baud = B200;     break;
        case 300:     baud = B300;     break;
        case 600:     baud = B600;     break;
        case 1200:    baud = B1200;    break;
        case 1800:    baud = B1800;    break;
        case 2400:    baud = B2400;    break;
        case 4800:    baud = B4800;    break;
        case 9600:    baud = B9600;    break;
        case 19200:   baud = B19200;   break;
        case 38400:   baud = B38400;   break;
        case 57600:   baud = B57600;   break;
        case 115200:  baud = B115200;  break;
        case 230400:  baud = B230400;  break;
        case 460800:  baud = B460800;  break;
        case 576000:  baud = B576000;  break;
        case 921600:  baud = B921600;  break;
        default:  printf("Invalid baud rate. Input baud rate is %ld.\n", baud_rate);
                  throw SerialException("Set baud rate failed.");
                  break;
    }
    ::cfsetispeed(&options, baud);
    ::cfsetospeed(&options, baud);

    // setup byte size
    options.c_cflag &= (tcflag_t) ~CSIZE;
    switch (byte_size) {
        case 5:  options.c_cflag |= CS5;  break;
        case 6:  options.c_cflag |= CS6;  break;
        case 7:  options.c_cflag |= CS7;  break;
        case 8:  options.c_cflag |= CS8;  break;
        default: printf("Invalid byte size. Input byte size is %d.\n", byte_size);
                 throw SerialException("Set byte size failed.");
                 break;
    }

    // setup parity
    switch (parity) {
        case 0:  options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);  break;
        case 1:  options.c_cflag |= (PARENB | PARODD);  break;
        case 2:  options.c_cflag &= (tcflag_t) ~(PARODD);
        options.c_cflag |= PARENB;  break;
        default: printf("Invalid parity. Input parity is %d.\n", parity);
                 throw SerialException("Set parity failed.");
                 break;
    }

    // setup stop bit
    switch (stop_bit) {
        case 1:  options.c_cflag &= (tcflag_t) ~CSTOPB;  break;
        case 2:  options.c_cflag |= CSTOPB;  break;
        case 3:  options.c_cflag |= CSTOPB;  break;  // one point five
        default: printf("Invalid stop bit. Input stop bit is %d.\n", stop_bit);
                 throw SerialException("Set stop bit failed.");
                 break;
    }

    // setup flow control
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
    options.c_cflag &= (tcflag_t) ~CRTSCTS;
    switch (flow_control) {
        case 0:  break;
        case 1:  options.c_iflag |= (IXON | IXOFF);  break;
        case 2:  options.c_cflag |= CRTSCTS;  break;
        default: printf("Invalid flow control. Input flow control is %d.\n", flow_control);
                 throw SerialException("Set flow control failed.");
                 break;
    }

    if (::tcsetattr(fd, TCSANOW, &options) != 0) {
        throw SerialException("Set port failed.");
    }
}

void SerialPort::sendData(const SendPack &send_pack) {
    if (!is_open) {
        throw SerialException("Send data failed. Port is not opened.");
    }

    uint8_t send_bytes[] = {0x55,        // frame head
                            0x00,        // mode
                            0x00, 0x00,  // yaw
                            0x00, 0x00,  // pitch
                            0x00,        // distance
                            0x00,        // check sum
                            0xA5};       // frame tail
    int16_t *data_ptr = (int16_t *)(send_bytes + 2);

    // send_bytes[1] = static_cast<uint8_t>(send_pack.mode);
    send_bytes[1] = static_cast<uint8_t>(0x50);
    data_ptr[0] = static_cast<int16_t>(send_pack.yaw * 100);
    data_ptr[1] = static_cast<int16_t>(send_pack.pitch * 100);
    send_bytes[6] = 0x10;
    send_bytes[7] = static_cast<uint8_t>(send_bytes[1] + send_bytes[2] + send_bytes[3] +
                                         send_bytes[4] + send_bytes[5] + send_bytes[6]);

    if (::write(fd, send_bytes, 9) == 9) {
        // printf("Send successfully.\n");
        // for (int i = 0; i < 8; ++i)  printf("%x\n", send_bytes[i]);
    } else {
        throw SerialException("Send data failed.");
    }
}

bool SerialPort::readData(ReadPack &read_pack) {
    if (!is_open) {
        throw SerialException("Read data failed. Port is not opened.");
    }

    uint8_t read_bytes[9] = {0, };

    while (!(read_bytes[0] == 0x55) && !(read_bytes[8] == 0xA5)) {
        for (int i = 0; i < 8; ++i)  read_bytes[i] = read_bytes[i+1];
        if (::read(fd, &read_bytes[8], 1) == 1) {
            // printf("Not read package. New byte: %x\n", send_bytes[8]);
        } else {
            throw SerialException("Read data failed.");
        }
    }

    uint8_t check_sum = static_cast<uint8_t>(read_bytes[1] + read_bytes[2] +
                                             read_bytes[3] + read_bytes[4] +
                                             read_bytes[5] + read_bytes[6]);
    if (check_sum == read_bytes[7]) {
        read_pack.enemy_color = read_bytes[1] > 10;
        read_pack.mode = static_cast<int>(read_bytes[2]);
        int16_t temp_pitch = (static_cast<int16_t>(read_bytes[3]) << 8) +
                              static_cast<int16_t>(read_bytes[4]);
        int16_t temp_yaw = (static_cast<int16_t>(read_bytes[5]) << 8) +
                            static_cast<int16_t>(read_bytes[6]);
        read_pack.pitch = static_cast<double>(temp_pitch) * 0.01;
        read_pack.yaw = static_cast<double>(temp_yaw) * 0.01;

        return true;
    } else {
        printf("check failed.\n");

        return false;
    }
}

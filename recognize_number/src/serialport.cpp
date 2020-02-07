/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

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
    fd_ = -1;
    is_open_ = false;
}

SerialPort::SerialPort(const string &port_name,
                       long baud_rate,
                       int byte_size,
                       int parity,
                       int stop_bit,
                       int flow_control) {
    port_name_ = port_name;
    baud_rate_ = baud_rate;
    byte_size_ = byte_size;
    parity_ = parity;
    stop_bit_ = stop_bit;
    flow_control_ = flow_control;
    is_open_ = false;
}

SerialPort::~SerialPort() {
   close();
}

void SerialPort::open() {
    if (port_name_.empty()) {
        throw SerialException("Open port failed. Port name is empty.");
    }

    if (is_open_) {
        throw SerialException("Open port failed. Port is already opened.");
    }

    fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ == -1) {
        throw SerialException("Open port failed. Bad file description.");
    }

    reconfigurePort();
    is_open_ = true;
}

void SerialPort::open(const string &port_name,
                      long baud_rate,
                      int byte_size,
                      int parity,
                      int stop_bit,
                      int flow_control) {
    port_name_ = port_name;
    baud_rate_ = baud_rate;
    byte_size_ = byte_size;
    parity_ = parity;
    stop_bit_ = stop_bit;
    flow_control_ = flow_control;

    if (port_name_.empty()) {
        throw SerialException("Open port failed. Port name is empty.");
    }

    if (is_open_) {
        throw SerialException("Open port failed. Port is already opened.");
    }

    fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ == -1) {
        throw SerialException("Open port failed. Bad file description.");
    }

    reconfigurePort();
    is_open_ = true;
}

bool SerialPort::isOpen() {
    return is_open_;
}

void SerialPort::close() {
    if (is_open_) {
        if (fd_ < 0) {
            throw SerialException("Close port failed. Bad file(open) description.");
        } else {
            int ret = ::close(fd_);
            if (ret == 0) {
                fd_ = -1;
                is_open_ = false;
            } else {
                throw SerialException("Close port failed. Bad file(close) description.");
            }
        }
    } else {
        throw SerialException("Close port failed. Port is not opened.");
    }
}

void SerialPort::setBaudRate(long baud_rate) {
    if (is_open_) {
        baud_rate_ = baud_rate;
        reconfigurePort();
    } else {
        throw SerialException("Set baud rate failed. Port is not opened.");
    }
}

long SerialPort::getBaudRate() {
    return baud_rate_;
}

void SerialPort::setByteSize(int byte_size) {
    if (is_open_) {
        byte_size_ = byte_size;
        reconfigurePort();
    } else {
        throw SerialException("Set byte size failed. Port is not opened.");
    }
}

int SerialPort::getByteSize() {
    return byte_size_;
}

void SerialPort::setParity(int parity) {
    if (is_open_) {
        parity_ = parity;
        reconfigurePort();
    } else {
        throw SerialException("Set parity failed. Port is not opened.");
    }
}

int SerialPort::getParity() {
    return parity_;
}

void SerialPort::setStopBit(int stop_bit) {
    if (is_open_) {
        stop_bit_ = stop_bit;
        reconfigurePort();
    } else {
        throw SerialException("Set stop bit failed. Port is not opened.");
    }
}

int SerialPort::getStopBit() {
    return stop_bit_;
}

void SerialPort::setFlowControl(int flow_control) {
    if (is_open_) {
        flow_control_ = flow_control;
        reconfigurePort();
    } else {
        throw SerialException("Set flow control failed. Port is not opened.");
    }
}

int SerialPort::getFlowControl() {
    return flow_control_;
}

void SerialPort::reconfigurePort() {
    if (fd_ == -1) {
        throw SerialException("Configure port failed. Bad file description.");
    }

    struct termios options;

    if (::tcgetattr(fd_, &options) < 0) {
        throw SerialException("tcgetattr error.");
    }

    options.c_cflag |= (tcflag_t) (CLOCAL | CREAD);
    options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG);
    options.c_oflag &= (tcflag_t) ~(OPOST);
    options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK | ISTRIP | INPCK);

    // setup baud rate
    speed_t baud;
    switch (baud_rate_) {
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
        default:  printf("Invalid baud rate. Input baud rate is %ld.\n", baud_rate_);
                  throw SerialException("Set baud rate failed.");
                  break;
    }
    ::cfsetispeed(&options, baud);
    ::cfsetospeed(&options, baud);

    // setup byte size
    options.c_cflag &= (tcflag_t) ~CSIZE;
    switch (byte_size_) {
        case 5:  options.c_cflag |= CS5;  break;
        case 6:  options.c_cflag |= CS6;  break;
        case 7:  options.c_cflag |= CS7;  break;
        case 8:  options.c_cflag |= CS8;  break;
        default: printf("Invalid byte size. Input byte size is %d.\n", byte_size_);
                 throw SerialException("Set byte size failed.");
                 break;
    }

    // setup parity
    switch (parity_) {
        case 0:  options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);  break;
        case 1:  options.c_cflag |= (PARENB | PARODD);  break;
        case 2:  options.c_cflag &= (tcflag_t) ~(PARODD);
        options.c_cflag |= PARENB;  break;
        default: printf("Invalid parity. Input parity is %d.\n", parity_);
                 throw SerialException("Set parity failed.");
                 break;
    }

    // setup stop bit
    switch (stop_bit_) {
        case 1:  options.c_cflag &= (tcflag_t) ~CSTOPB;  break;
        case 2:  options.c_cflag |= CSTOPB;  break;
        case 3:  options.c_cflag |= CSTOPB;  break;  // one point five
        default: printf("Invalid stop bit. Input stop bit is %d.\n", stop_bit_);
                 throw SerialException("Set stop bit failed.");
                 break;
    }

    // setup flow control
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
    options.c_cflag &= (tcflag_t) ~CRTSCTS;
    switch (flow_control_) {
        case 0:  break;
        case 1:  options.c_iflag |= (IXON | IXOFF);  break;
        case 2:  options.c_cflag |= CRTSCTS;  break;
        default: printf("Invalid flow control. Input flow control is %d.\n", flow_control_);
                 throw SerialException("Set flow control failed.");
                 break;
    }

    if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
        throw SerialException("Set port failed.");
    }
}

void SerialPort::sendData(double yaw, double pitch) {
    if (!is_open_) {
        throw SerialException("Send data failed. Port is not opened.");
    }

    uint8_t send_bytes[] = {0x55, 0xAA,  // frame head
                            0x00, 0x00,  // yaw
                            0x00, 0x00,  // pitch
                            0x00,        // check sum
                            0xA5};       // frame tail
    int16_t *data_ptr = (int16_t *)(send_bytes + 2);

    data_ptr[0] = static_cast<int16_t>(yaw * 100);
    data_ptr[1] = static_cast<int16_t>(pitch * 100);
    send_bytes[6] = static_cast<uint8_t>(send_bytes[2] + send_bytes[3] +
                                         send_bytes[4] + send_bytes[5]);

    if (::write(fd_, send_bytes, 8) == 8) {
        // printf("Send successfully.\n");
        // for (int i = 0; i < 8; ++i)  printf("%x\n", send_bytes[i]);
    } else {
        throw SerialException("Send data failed.");
    }
}

bool SerialPort::readData(int &enemy_color, int &mode) {
    if (!is_open_) {
        throw SerialException("Read data failed. Port is not opened.");
    }

    uint8_t last_package[5] = {0, };

    while (!isReadPackage(last_package)) {
        for (int i = 0; i < 4; ++i)  last_package[i] = last_package[i+1];
        if (::read(fd_, &last_package[4], 1) == 1) {
            // printf("Not read package. New byte: %x\n", send_bytes[4]);
        } else {
            throw SerialException("Read data failed.");
        }
    }

    enemy_color = 1;
    mode = 1;
    return true;
}

bool SerialPort::isReadPackage(uint8_t *pack) {
    static const uint8_t check_data[3] = {0x55, 0xAA, 0x0D};  // frame head and frame tail
    bool result = true;

    result = result &&
             (pack[0] == check_data[0]) &&
             (pack[1] == check_data[1]) &&
             (pack[4] == check_data[2]);

    return result;
}

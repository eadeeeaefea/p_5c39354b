/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.3.18

 Detail: 实现串口相关参数的设定和初始化，并通过协调好的通信协议进行数据的打包和解包，完成数据
         的收发。
 *****************************************************************************/

#include "serialport.h"


SerialPort::SerialPort() {
    baud_ = B115200;
    fd_port_ = -1;
}

SerialPort::~SerialPort() {
    closePort(fd_port_);
}

void SerialPort::init(const char* port_name, int baud_rate) {
    switch (baud_rate) {
        case 0:       baud_ = B0;       break;
        case 50:      baud_ = B50;      break;
        case 75:      baud_ = B75;      break;
        case 110:     baud_ = B110;     break;
        case 134:     baud_ = B134;     break;
        case 150:     baud_ = B150;     break;
        case 200:     baud_ = B200;     break;
        case 300:     baud_ = B300;     break;
        case 600:     baud_ = B600;     break;
        case 1200:    baud_ = B1200;    break;
        case 1800:    baud_ = B1800;    break;
        case 2400:    baud_ = B2400;    break;
        case 4800:    baud_ = B4800;    break;
        case 9600:    baud_ = B9600;    break;
        case 19200:   baud_ = B19200;   break;
        case 38400:   baud_ = B38400;   break;
        case 57600:   baud_ = B57600;   break;
        case 115200:  baud_ = B115200;  break;
        case 230400:  baud_ = B230400;  break;
        case 460800:  baud_ = B460800;  break;
        case 576000:  baud_ = B576000;  break;
        case 921600:  baud_ = B921600;  break;
        default:    break;
    }

    while (1) {
        fd_port_ = configurePort(openPort(port_name));
        if (fd_port_ >= 0)  break;
    }

}

bool SerialPort::sendData(double yaw, double pitch) {

    uint8_t send_bytes[] = {0x55, 0x52,  // frame head
                            0x00, 0x00,  // yaw
                            0x00, 0x00,  // pitch
                            0x00,        // control mode
                            0x00,        // chect sum
                            0xA5};       // frame tail
    int16_t temp_yaw, temp_pitch;
    uint8_t temp_mode, check_sum;

    temp_yaw = static_cast<int16_t>(yaw * 100);
    temp_pitch = static_cast<int16_t>(pitch * 100);
    temp_mode = static_cast<uint8_t>(2);
    check_sum = static_cast<uint8_t>(temp_yaw + temp_pitch + temp_mode);

    int16_t *data_ptr = (int16_t *)(send_bytes + 2);
    data_ptr[0] = temp_yaw;
    data_ptr[1] = temp_pitch;
    send_bytes[7] = temp_mode;
    send_bytes[8] = check_sum;

    if (write(fd_port_, send_bytes, 9) == 9) {
        // printf("send successfully.\n");
        // for (int i = 0; i<9; i++)
        //     printf("%x\n", send_bytes[i]);
        return true;
    }

    return false;
}

bool SerialPort::readData(int &enemy_color, int &mode) {
    uint8_t last_package[5] = {0, };

    while (!isReadPackage(last_package)) {
        for (int i = 0; i < 4; ++i)
            last_package[i] = last_package[i+1];
        read(fd_port_, &last_package[4], 1);
        // printf("%x\n", last_package[4]);
    }

    return false;
}

bool SerialPort::isReadPackage(uint8_t *buf) {
    static const uint8_t read_package[5] = {0x55, 0x00, 0x00, 0xA5, 0x0D};  // 读取的数据包，按协议进行修改
    int cnt = 0;

    for (int i = 0; i < 5; ++i) {
        if (i == 0 || i == 3 || i == 4)
            if (buf[i] == read_package[i])
                cnt++;
    }

    if (cnt == 3) {
        return true;
    } else {
        return false;
    }
}

int SerialPort::openPort(const char* port_name) {

    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        printf("Open port failed.\n");
    } else {
        printf("Open port successfully.\n");
    }

    return fd;
}

int SerialPort::configurePort(int fd) {
    struct termios port_settings;          // structure to set basic parameters of serial port
    cfsetispeed(&port_settings, baud_);    // set baud rate
    cfsetospeed(&port_settings, baud_);

    // port_settings.c_cflag |= CLOCAL | CREAD;  // 本地连接和接受使能
    // port_settings.c_iflag &= ~ICRNL;  // 禁止将输入中的回车翻译为新行 (除非设置了 IGNCR)
    // port_settings.c_iflag &= ~ISTRIP;  // 禁止将所有接收的字符裁减为7比特
    port_settings.c_cflag &= ~PARENB;  // 无校验
    port_settings.c_cflag &= ~CSTOPB;  // 1位停止位
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;  // 8位数据位
    port_settings.c_cflag &= ~CRTSCTS;  // 禁止硬件流控

    if (tcsetattr(fd, TCSANOW, &port_settings) != 0) {
        printf("set port failed!\n");
        return -1;
    }

    // struct stat port_stat;
    // struct flock port_lock;
    // fstat(fd, &port_stat);
    // port_lock.l_len = port_stat.st_size;
    // port_lock.l_pid = getpid();
    // port_lock.l_start = 0;
    // port_lock.l_type = F_WRLCK;
    // port_lock.l_whence = SEEK_SET;
    // printf("current pid = %d\n", port_lock.l_pid);
    // while (fcntl(fd, F_SETLK, &port_lock) < 0) {
    //     struct flock temp_lock;
    //     temp_lock = port_lock;
    //     temp_lock.l_type = F_WRLCK;
    //     temp_lock.l_pid = -1;
    //     fcntl(fd, F_GETLK, &temp_lock);
    //
    //     switch (temp_lock.l_type) {
    //         case F_RDLCK:
    //             printf("Read lock, pid = %d\n", temp_lock.l_pid);
    //             break;
    //         case F_UNLCK:
    //             printf("Unlocked, pid = %d\n", temp_lock.l_pid);
    //             break;
    //         default:
    //             break;
    //     }
    // }
    // printf("lock port successfully.\n");

    return fd;
}

void SerialPort::closePort(int fd) {
    close(fd);
}

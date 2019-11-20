# HERORM2020
该代码主要为RoboMaster2020赛季HERO团队视觉组交流使用。

## 环境依赖
- ubuntu 16.04及以上
- opencv3.4及以上
- cmake3.5.1及以上

## 编译运行
```sh
git clone ${this project}
mv p_5c39354b-master HERORM2020
cd HERORM2020
mkdir build
cd build
cmake ..
make
./HERORM2020
```

## update
- 2019.10.13
    - 扩充预处理的图片，解决ROI后预处理图片过小难以准确调整trackbar的问题
    - 更新SerialPort的封装方式，丰富串口设置的相关函数，预留异常处理接口，解决ttyTHS2使用时出现的问题
    - 更新MVCamera的封装方式，预留异常处理接口

- 2019.10.16
    - 新增串口异常处理类和异常
    - 新增工业摄像头异常处理类和异常
    - 新增串口和工业摄像头发生异常时相应的处理代码
    - 新增在/dev/ttyUSB0～2中搜索端口打开串口的代码，解决串口号改变引起的程序执行问题
    - 完善monitor.sh的编写

- 2019.10.17
    - 新增HSV预处理方案

- 2019.11.20
    - 完成新通信协议的编写和调试
    - 更新短时间内匀速运动模型预测代码的编写
    - 通过扩充图片解决trackbar调试时ROI后预处理图像过小导致trackbar难以使用的问题

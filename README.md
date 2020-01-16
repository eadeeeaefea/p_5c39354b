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
- 2019.11.25
   -   对所有函数进行了更新，包括弹道解算，运动预测，以及增加了函数预处理方式，对很多类都作了小修改
- 2019.11.27
   -   对运动预测作了改进，并对所有代码做了一些规范调整
- 2019.11.28
   -   对角度解算作了改动更新
- 2019.12.05
   -   更新了运动预测算法
- 2019.12.30
   -   整理并整合了anglesolver算法
   -   将各个功能模块均在小电脑上测试过可正常运行
   -   can等通信协议已经校验整合完毕
   -   步兵代码整体架构完成，开始上车调参
   -   根据新车参数修正targetsolver中建立的坐标系的参数
- 2020.01.16
   -   上传步兵使用的最新代码，但由于在拍摄中期形态视频的风车调试问题，所有有关风车的代码请不要放入代码中，universalpredict是最新的预测代码，但不能使用，原因是整体思路未确定，imageprocess也未用到，在修改中，以获得更好的预处理效果
   -   弹道解算未加任何参数，未改动workspace中的架构和逻辑

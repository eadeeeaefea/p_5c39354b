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
- 2019.10.21.
    - 上传经测试的弹道计算代码：
      主要算法是先将坐标旋转变换至ｙ轴竖直的坐标系，再进行二分法解算（速度最快、准确度高);
      附加未增加旋转变换的二分法解算和速度矢量三角推公式算法的代码供备用（效果与旋转变换过的无明显差异，解算速度较慢）
      
- 2019.10.23.
    - 按照编程规范修改代码格式，保证编译无错；
    - 由于通讯函数尚未写好，弹道计算函数在workspace中调用的是无坐标转换的算法
    
- 2019.12.29.
    - 上传了整合了最新can和串口通信协议的代码，可以正确在小电脑上运行大风车、装甲板和上位机。
    
- 2020.01.18.
    - 上传了截至中期形态视频的最终版步兵代码，主要修改如下：
      -大风车代码部分参数改动（箭头面积、旋转角速度等）以防止误识别并更精准的打击能量机关。但是从中期视频拍摄效果来看这个算法本身有问题，计划寒假后重写代码；
      -串口通信和can通信部分已根据通信需要调试好；
      -弹道解算整合了所有6种算法，可随时更换。

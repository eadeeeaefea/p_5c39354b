# HERO --- LHT
---
Used for HERO-RM 20 Season sentinel robot

## Details

1. Seperate gimbles
    Since we get two gimbles on sentinel this season, so there are seperately two mostly the same program in folder *chassis & gimble*

## Update

* 2019.10.11
    * A plot program powered by qt stored in folder Plot
<br></br>

* 2019.10.20
    * Pack up the program and encapsulate QT into an app. For detailed usage, please see readme folder app
<br></br>

* 2019.12.8
    * Optimize QT plot communication protocol
    * CAN communication demo uploaded
    * Sending plotting data through serial port
    * Choose MVCamera by unique camera id 
<br></br>

* 2019.12.22
    * add *app* using notice
      * raise right before using 
        * `chmod 666 AppRun` 
    * remove some redundant folder
<br></br>

* 2019.12.29
    * modify new protocol with stm32 in both CAN & serial
    * fix some logical errors of macro definition in workspace.cpp 
    * fix bug of sendPlot in serialport.cpp
<br></br>

* 2020.1.15
    * this version serves as first-generation sentinel code
    * remove **CAN** module coz not used
    * change `openSerial()` function in `workspace.cpp` 
        * *reason*: there are two serial ports used on sentinel
        * *method*: use **udev** module in Linux to remap two serials seperately
    * optimize **TRACKBAR** logic
        * *reason:* mostly, we do not use trackbar in armordetector while coping with robots
        * *method:* like TEST macro definition in `bast.h`, define **TRACKBAR** in three ways
        * 0 --> offset only;  1 --> armordetector only; 2 --> both
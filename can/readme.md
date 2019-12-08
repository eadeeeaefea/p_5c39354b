# CAN Communication Demo

## Quick start

- To test this demo, you can compile code, and call *main.cpp* by:
    1. mkdir build
    2. cd build
    3. cmake .. 
    4. make
    5. **sudo** ./RMTest_CAN

## Param you can modify

### Choose you device

Change macro definition in *base.h*

```C++
#define USE_CAN 0
```
0 --> use can0   1 --> use can1

### Changes for incoming packets

- Each packet in CAN com has its own id, like header bytes in a packet from serail port 
    
- To change packet id you wanna receive, there are two parameters you can modify

1. Filter & number of id
    * CAN uses filters to filter incoming ids. Each filter serves for one incoming id. So you must first modify  the number of filter in *cannode.h*, which is a private member of this class

    * But it's already done, what you only need to do is change the number by revising a macro def in *cannode.h*
    
```C++
#define ID_NUM  1
```

2. Modify receiving id in *cannode.h*

```C++
unsigned int id_rcv_[ID_NUM] = {0x301};
```

### Changes for packets to send

* The same as receiving above

### NOTICE

1. Sending frequency can not be too fast, so you can find sleep() function in test demo

2. To test this single demo, you must use super admin(sudo) to run as shown at the top.
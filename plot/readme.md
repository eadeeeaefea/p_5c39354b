# QT plotter

## Using this demo

* This demo receives data from serial port and the protocol follows:
    
```c++
    pack[0] --> header = 0x31
    pack[1] --> plot_type  from 0 to 4
    pack[2] --> curve number <= 3
    pack[3] & pack[4] --> value1
    pack[5] & pack[6] --> value2
    pack[7] & pack[8] --> value3
    pack[9] --> checksum
    pack[10] --> tail = 0xE4
```

1. `plot_type` : you can choose your curve name by this parameter

```c++
enum PLOT_TYPE
{
    TYPE_X = 0,
    TYPE_Y = 1,
    TYPE_Z = 2,
    TYPE_YAW = 3,
    TYPE_PITCH = 4
};
```    

2. `curve number` : the number of curve you wanna to plot       (**must <= 3**)
    
    * e.g. if your `plot_type = 1`, and you give `curve_number = 3`, then this demo will plot axes `Y` & `Z` & `YAW`

3. `chechsum` : Sum up all data(***except tail & header***) and convert it to uint8_t


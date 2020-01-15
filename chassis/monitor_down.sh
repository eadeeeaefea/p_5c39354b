#!/bin/bash
## Author: Wang Xiaoyan on 2019.10.15
## using:
### Ubuntu 16.04: rc.local
### Ubuntu 18.04: systemd

cd /home/hero/RM2020/chassis/build
ls
make clean && make

count=0
while [ true ]; do
    status=`ps -ef | grep HERORM2020_DOWN | grep -v grep | wc -l`
    if [ $status -eq 0 ]; then
        echo "HERORM2020_DOWN is not running. Restarting..."
        ./HERORM2020_DOWN
        count=count+1
        if [ $count -gt 10 ]; then
            reboot
        fi
    fi
    sleep 3
done

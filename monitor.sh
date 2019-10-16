#!/bin/bash
## Author: Wang Xiaoyan on 2019.10.15
## using:
### Ubuntu 16.04: rc.local
### Ubuntu 18.04: systemd

cd HERORM2020/build
make clean && make -j4

count=0
while [ true ]; do
    status=`ps -ef | grep HERORM2020 | grep -v grep | wc -l`
    if [ $status -eq 0 ]; then
        echo "HERORM2020 is not running. Restarting..."
        ./HERORM2020
        count=count+1
        if [ $count -gt 10 ]; then
            reboot
        fi
    fi
    sleep 3
done

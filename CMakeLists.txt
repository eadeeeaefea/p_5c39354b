cmake_minimum_required(VERSION 3.0)
PROJECT(HERORM2020)

set(CMAKE_CXX_STANDARD 14)
set(OpenCV_DIR /usr/local/share/OpenCV)

find_package(OpenCV REQUIRED)
include_directories(./include
        ${OpenCV_INCLUDE_DIRS}
        )

add_executable(HERORM2020
        src/main.cpp
        include/base.h
        include/timer.h
        src/timer.cpp
        include/armordetector.h
        src/armordetector.cpp
        include/robotdetector.h
        src/robotdetector.cpp
        include/robottracker.h
        src/robottracker.cpp
        include/armorchecker.h
        src/armorchecker.cpp
        include/workspace.h
        src/workspace.cpp
        include/serialport.h
        src/serialport.cpp
        include/DxImageProc.h
        include/GxIAPI.h
        include/dhcamera.h
        src/dhcamera.cpp
        )

target_link_libraries(HERORM2020
        ${OpenCV_LIBRARIES}
        -pthread
        # -lMVSDK
        # /lib/libMVSDK.so
        -lgxiapi
)



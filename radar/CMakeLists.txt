cmake_minimum_required(VERSION 3.0)
PROJECT(HERORM2020)

set(CMAKE_CXX_STANDARD 11)
set(OpenCV_DIR /usr/local/share/OpenCV)
#set(OPENCV_DIR /home/luzhan/软件/opencv-3.4.3)
set(Torch_DIR /media/stayalive/E29CC67C9CC64AAD/me/Hero/radar/libtorch/share/cmake/Torch)

find_package(Torch REQUIRED)
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
        ${TORCH_LIBRARIES}
        )

target_link_libraries(HERORM2020
        ${OpenCV_LIBRARIES}
        -pthread
        # -lMVSDK
        # /lib/libMVSDK.so
        -lgxiapi
)



cmake_minimum_required(VERSION 3.10)
project(Reconstruction)

set(CMAKE_CXX_STANDARD 14)
set(OpenCV_DIR /usr/local/share/OpenCV)

find_package(OpenCV REQUIRED)

include_directories(./include
        ${OpenCV_INCLUDE_DIRS}
        )

add_executable(Reconstruction 
        src/main.cpp
        src/recognize.cpp
        include/recognize.h
        include/stereo.h
        src/stereo.cpp
        include/timer.h
        src/timer.cpp
        src/kalman.cpp
        include/kalman.h
        )

target_link_libraries(Reconstruction
        ${OpenCV_LIBRARIES}
        -pthread
        -lMVSDK
        /lib/libMVSDK.so
        )

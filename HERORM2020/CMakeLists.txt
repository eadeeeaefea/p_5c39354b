cmake_minimum_required(VERSION 3.5.1)
project(HERORM2020)

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
    include/mvcamera.h
    src/mvcamera.cpp
    include/serialport.h
    src/serialport.cpp
    include/armordetector.h
    src/armordetector.cpp
    include/targetsolver.h
    src/targetsolver.cpp
    include/anglesolver.h
    src/anglesolver.cpp
    include/runesolver.h
    src/runesolver.cpp
    include/predictor.h
    src/predictor.cpp
    include/cannode.h
    src/cannode.cpp
    include/an_predictor.h
    src/an_predictor.cpp
    include/workspace.h
    src/workspace.cpp
)

target_link_libraries(HERORM2020
    ${OpenCV_LIBRARIES}
    -pthread
    -lMVSDK
    /lib/libMVSDK.so
)

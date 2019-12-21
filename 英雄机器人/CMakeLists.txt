cmake_minimum_required(VERSION 3.5.1)
project(HERORM2020_Hero)

set(CMAKE_CXX_STANDARD 14)
#set(OpenCV_DIR /usr/local/share/OpenCV)
set(OpenCV_DIR /home/luzhan/软件/opencv-3.4.3)
find_package(OpenCV REQUIRED)

# 添加源文件
add_executable(HERORM2020_Hero
    src/main.cpp
    src/mvcamera.cpp
    src/serialport.cpp
    src/armordetector.cpp
    src/targetsolver.cpp
    src/anglesolver.cpp
    src/predictor.cpp
    src/workspace.cpp
)

# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS} ./include)
include_directories(include)
# 添加链接库
target_link_libraries(HERORM2020_Hero
    ${OpenCV_LIBRARIES}
    -pthread
    -lMVSDK
    /lib/libMVSDK.so
)

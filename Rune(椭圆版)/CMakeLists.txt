cmake_minimum_required(VERSION 3.7)
project(Rune_)

set(CMAKE_CXX_STANDARD 11)
set(OPENCV_DIR /home/luzhan/软件/opencv-3.4.3)

find_package(OpenCV 3.4.3 REQUIRED)
# 添加源程序
add_executable(Rune_
		       src/main.cpp
		       src/timer.cpp
			   src/energy.cpp
			   src/videoio.cpp
			   src/draw.cpp
			   src/mvcamera.cpp
			   src/anglesolver.cpp
			   src/serialport.cpp
			   src/workspace.cpp)
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS} ./include)
include_directories(include)

target_link_libraries(Rune_
					  ${OpenCV_LIBS}
					  -pthread
					  -lMVSDK
					  /lib/libMVSDK.so)

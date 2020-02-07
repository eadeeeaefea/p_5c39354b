cmake_minimum_required(VERSION 3.5.0)
project(number)

set(CMAKE_CXX_STANDARD 14)
set(OpenCV_DIR /usr/local/share/OpenCV)

add_compile_options(-std=c++14)

find_package(OpenCV REQUIRED)
include_directories(./include
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_NAME}

)

add_library(${PROJECT_NAME}
        include/base.h
        include/timer.h
        include/mvcamera.h
        src/mvcamera.cpp
        include/serialport.h
        src/serialport.cpp
        include/armordetector.h
        src/armordetector.cpp
        )

add_executable(recognizetest src/recognizetest.cpp)
target_link_libraries(recognizetest ${OpenCV_LIBS}
  ${PROJECT_NAME}
)

add_executable(getdataset src/getdataset.cpp)
target_link_libraries(getdataset ${OpenCV_LIBS}
${PROJECT_NAME}
)

target_link_libraries(number
        ${OpenCV_LIBRARIES}
        -lMVSDK
        /lib/libMVSDK.so
        )
cmake_minimum_required(VERSION 3.5.1)
project(VideoCap)

set(CMAKE_CXX_STANDARD 14)

if(UNIX)
  # If Linux, add rt and pthread
  set(rt_LIBRARIES rt)
  set(pthread_LIBRARIES pthread)
  include_directories(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread
  )
endif()

set(HERORM2019_LIB
  src/main.cpp
  src/camera.cpp
  include/camera.h
)

set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
  ./include
)

# add_library(${PROJECT_NAME}
#   ${serial_SRCS}
#   ${HERORM2019_LIB}
# )

add_executable(${PROJECT_NAME}
  ${HERORM2019_LIB}
)

if(UNIX)
  target_link_libraries(${PROJECT_NAME} rt pthread
    ${OpenCV_LIBRARIES}
    # libopencv_world.so
)
endif()

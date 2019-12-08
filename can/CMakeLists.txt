cmake_minimum_required(VERSION 3.5.1)
project(RMTest_CAN)

set(CMAKE_CXX_STANDARD 14)

if(UNIX)
  # If Linux, add rt and pthread
  set(rt_LIBRARIES rt)
  set(pthread_LIBRARIES pthread)
  include_directories(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS header
    DEPENDS rt pthread
  )
endif()

set(RMTEST_LIB 
  src/main
  src/cannode.cpp
  include/cannode.h
)

include_directories(./include)

add_executable(${PROJECT_NAME}
  ${RMTEST_LIB}
)

if(UNIX)
  target_link_libraries(${PROJECT_NAME} rt pthread
)
endif()


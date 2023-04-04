cmake_minimum_required(VERSION 3.15)
project(OpenCV)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LOGS REQUIRED Logs)
pkg_check_modules(XBEE REQUIRED Xbee)

include_directories(${OpenCV_INCLUDE_DIRS} ${LOGS_INCLUDE_DIRS} ${XBEE_INCLUDE_DIRS})

add_executable(main src/main.cpp
        src/serialib.cpp include/serialib.h
        src/estimator.cpp include/estimator.h)

target_link_libraries(main ${OpenCV_LIBRARIES} ${LOGS_LIBRARIES} ${XBEE_LIBRARIES})
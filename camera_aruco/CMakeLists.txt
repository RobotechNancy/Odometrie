cmake_minimum_required(VERSION 3.15)
project(ArUCO)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_check_modules(XBee REQUIRED XBee)

include_directories(${OpenCV_INCLUDE_DIRS} includes)

add_executable(main src/main.cpp includes/camera.h
        src/calibration.cpp src/estimation.cpp
        src/camera.cpp includes/estimation.h includes/calibration.h)

target_link_libraries(main ${OpenCV_LIBRARIES} ${XBee_LIBRARIES})

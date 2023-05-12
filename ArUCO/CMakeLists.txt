cmake_minimum_required(VERSION 3.25)
project(ArUCO)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_check_modules(XBee REQUIRED XBee)

include_directories(${OpenCV_INCLUDE_DIRS} includes)

add_executable(ArUCO src/main.cpp includes/tags.h
        includes/camera.h src/camera.cpp
        src/calibration.cpp includes/calibration.h
        src/estimation.cpp includes/estimation.h)

target_link_libraries(ArUCO ${OpenCV_LIBRARIES} ${XBee_LIBRARIES})

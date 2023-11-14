cmake_minimum_required(VERSION 3.18)
project(ArUCO)

set(CMAKE_CXX_STANDARD 17)
include_directories(includes)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(XBee REQUIRED)

add_executable(ArUCO
        main.cpp
        includes/camera.h src/camera.cpp
        includes/calibration.h src/calibration.cpp
        includes/estimation.h src/estimation.cpp
)

target_link_libraries(ArUCO ${OpenCV_LIBRARIES} ${XBee_LIBRARIES} Threads::Threads)
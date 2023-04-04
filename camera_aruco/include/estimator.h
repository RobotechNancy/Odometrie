//
// Created by mrspaar on 1/17/23.
//

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <mutex>
#include <robotech/xbee.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#define REF_ARUCO_ID 47
#define BREAK_KEY 27
#define IMG_DELAY 10

class Estimator {
private:
    cv::Vec3d origin;
    std::vector<int> ids;
    cv::VideoCapture cap;
    float marker_length = 0.020;
    std::vector<cv::Vec3d> rvecs, tvecs;

    XBee xbee;
    std::mutex mtx;
    cv::aruco::ArucoDetector detector;
public:
    Estimator();

    int calibrate(int markersX, int markersY, float markerLength, float markerSeparation);
    int start();
    char *to_bytes();
};

void createBoard(int markersX, int markersY, int markerLength, int markerSeparation);

#endif //ESTIMATOR_H

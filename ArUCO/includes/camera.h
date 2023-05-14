//
// Created by mrspaar on 4/25/23.
//

#ifndef OPENCV_CAMERA_H
#define OPENCV_CAMERA_H

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>


class Camera {
public:
    explicit Camera(const cv::FileStorage& configFile);

    cv::VideoCapture cap;
    cv::aruco::ArucoDetector detector;
};


#endif //OPENCV_CAMERA_H

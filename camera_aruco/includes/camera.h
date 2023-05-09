//
// Created by mrspaar on 4/25/23.
//

#ifndef OPENCV_CAMERA_H
#define OPENCV_CAMERA_H

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#define REF_ARUCO_ID 47
#define DICTIONNARY cv::aruco::DICT_4X4_50


class Camera {
public:
    Camera();

    cv::VideoCapture cap;
    cv::aruco::ArucoDetector detector;
};


#endif //OPENCV_CAMERA_H

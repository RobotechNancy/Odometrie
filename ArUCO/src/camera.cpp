//
// Created by mrspaar on 4/25/23.
//

#include <iostream>
#include "camera.h"


Camera::Camera(const cv::FileStorage& configFile) {
    cap = cv::VideoCapture(0);

    if (!cap.isOpened()) {
        std::cerr << "Impossible d'ouvrir la caméra" << std::endl;
        exit(-1);
    }

    cv::FileStorage fs(
            configFile["detector_params_path"],
            cv::FileStorage::READ
    );

    if (!fs.isOpened()) {
        std::cerr << "Impossible d'ouvrir \"detector_params.yml\"" << std::endl;
        exit(-1);
    }

    cv::aruco::DetectorParameters detectorParams;
    detectorParams.readDetectorParameters(fs.root());
    fs.release();

    detector.setDictionary(
            cv::aruco::getPredefinedDictionary((int) configFile["dictionary"])
    );
    detector.setDetectorParameters(detectorParams);
}

//
// Created by mrspaar on 4/25/23.
//

#include "camera.h"


Camera::Camera() {
    cap = cv::VideoCapture(0);

    if (!cap.isOpened()) {
        std::cerr << "Impossible d'ouvrir la caméra" << std::endl;
        exit(-1);
    }

    cv::FileStorage fs("../detector_params.yml", cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "Impossible d'ouvrir \"detector_params.yml\"" << std::endl;
        exit(-1);
    }

    cv::aruco::DetectorParameters detectorParams;
    detectorParams.readDetectorParameters(fs.root());
    fs.release();

    detector.setDetectorParameters(detectorParams);
    detector.setDictionary(getPredefinedDictionary(DICTIONNARY));
}

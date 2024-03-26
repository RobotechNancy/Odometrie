//
// Created by mrspaar on 4/25/23.
//

#include <iostream>
#include "camera.h"


Camera::Camera(const cv::FileStorage& configFile) {
    // 0 -> utiliser la caméra par défaut
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

    // On récupère les paramètres du détecteur à partir de "detector_params_path"
    cv::aruco::DetectorParameters detectorParams;
    detectorParams.readDetectorParameters(fs.root());
    fs.release();

    // Dictionnaire contenant les marqueurs détectables
    detector.setDictionary(cv::aruco::getPredefinedDictionary((int) configFile["dictionary"]));
    detector.setDetectorParameters(detectorParams);
}

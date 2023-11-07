//
// Created by mrspaar on 4/29/23.
//

#ifndef OPENCV_CALIBRATION_H
#define OPENCV_CALIBRATION_H

#include <opencv2/core/persistence.hpp>


// Créer un fichier PNG contenant un plateau de marqueurs
void boardToPng(const cv::FileStorage& configFile);

// Calibrer une caméra (déterminer sa matrice et ses coefficients de distortion)
uint8_t calibrate(const cv::FileStorage& configFile);


#endif //OPENCV_CALIBRATION_H

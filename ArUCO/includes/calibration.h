//
// Created by mrspaar on 4/29/23.
//

#ifndef OPENCV_CALIBRATION_H
#define OPENCV_CALIBRATION_H

#include <opencv2/core/persistence.hpp>


/*!
 * @brief Créer un fichier PNG contenant un plateau de marqueurs
 * @param configFile Fichier de configuration
 */
void boardToPng(const cv::FileStorage& configFile);


/*!
 * @brief Calibrer une caméra (déterminer sa matrice et ses coefficients de distortion)
 * @param fs Fichier de configuration
 * @return
 */
uint8_t calibrate(const cv::FileStorage& configFile);


#endif //OPENCV_CALIBRATION_H

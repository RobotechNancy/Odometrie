//
// Created by mrspaar on 4/29/23.
//

#ifndef OPENCV_CALIBRATION_H
#define OPENCV_CALIBRATION_H

#include "camera.h"


/*!
 * @brief Créer un fichier PNG contenant un plateau de marqueurs
 * @param path      Chemin du fichier PNG à créer
 * @param markersX  Nombre de marqueurs en X
 * @param markersY  Nombre de marqueurs en Y
 * @param markerLen Longueur d'un marqueur (en m)
 * @param markerSep Longueur de séparation entre deux marqueurs (en m)
 */
void boardToPng(const std::string &path, uint8_t markersX, uint8_t markersY, uint8_t markerLen, uint8_t markerSep);


/*!
 * @brief Calibrer une caméra (déterminer sa matrice et ses coefficients de distortion)
 * @param camera    Caméra à calibrer
 * @param markersX  Nombre de marqueurs en X
 * @param markersY  Nombre de marqueurs en Y
 * @param markerLen Longueur d'un marqueur (en m)
 * @param markerSep Longueur de séparation entre deux marqueurs (en m)
 * @return
 */
uint8_t calibrate(Camera camera, uint8_t markersX, uint8_t markersY, uint8_t markerLen, uint8_t markerSep);

#endif //OPENCV_CALIBRATION_H

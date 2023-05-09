//
// Created by mrspaar on 4/29/23.
//

#ifndef OPENCV_CALIBRATION_H
#define OPENCV_CALIBRATION_H

#include "camera.h"


/*!
 * @brief Classe pour la calibration de la caméra
 * @details Calibration(markersX, markersY, markerLen, markerSep)
 */
class Calibration {
public:
    explicit Calibration(uint8_t markersX, uint8_t markersY, uint8_t markerLen, uint8_t markerSep);

    uint8_t start();
    void boardToPng(const std::string &path);
private:
    Camera camera;

    uint8_t markersX;
    uint8_t markersY;
    uint8_t markerLen;
    uint8_t markerSep;
};

#endif //OPENCV_CALIBRATION_H

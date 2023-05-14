//
// Created by mrspaar on 4/29/23.
//

#ifndef OPENCV_ESTIMATION_H
#define OPENCV_ESTIMATION_H

#include "camera.h"
#include "robotech/xbee.h"


/*!
 * @brief Classe pour l'estimation de la position des marqueurs
 * @details Estimation(markerLen)
 */
class Estimation {
public:
    explicit Estimation(const cv::FileStorage& configFile);

    void update();
    void send(XBee& xbee, uint8_t dest);
private:
    Camera camera;

    float markerLen;
    int refMarkerId;

    cv::Vec3d origin;
    cv::Mat image, cameraMatrix, distCoeffs;

    std::mutex mtx;
    std::vector<int> ids;
    std::vector<cv::Vec3d> rVecs, tVecs;
    std::vector<std::vector<cv::Point2f>> corners;
};


#endif //OPENCV_ESTIMATION_H

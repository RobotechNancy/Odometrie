//
// Created by mrspaar on 4/29/23.
//

#ifndef OPENCV_ESTIMATION_H
#define OPENCV_ESTIMATION_H

#include "camera.h"
#include "robotech/xbee.h"


class Estimation {
public:
    explicit Estimation(const cv::FileStorage& configFile);  // Prend le chemin vers config.yml en paramètre
    void update();                                           // Mettre à jour tVecs et rVecs
    void send(XBee& xbee, uint8_t dest);                     // Envoyer les données à un destinataire
private:
    Camera camera;


    float markerLen;
    int refMarkerId;
    cv::Vec3d refMarkerPos;

    cv::Vec3d origint, originr;

    cv::Mat image, cameraMatrix, distCoeffs;

    std::mutex mtx;                                          // Évite les accès simultanés à tVecs et rVecs
    std::vector<int> ids;                                    // ⎡
    std::vector<cv::Vec3d> rVecs, tVecs;                     // | Données de la dernière détection
    std::vector<std::vector<cv::Point2f>> corners;           // ⎣
};


#endif //OPENCV_ESTIMATION_H


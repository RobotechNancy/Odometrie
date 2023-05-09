//
// Created by mrspaar on 4/29/23.
//

#ifndef OPENCV_ESTIMATION_H
#define OPENCV_ESTIMATION_H

#include "camera.h"

typedef std::function<void(const std::vector<int>&, const std::vector<cv::Vec3d>&, const std::vector<cv::Vec3d>&)> callback_t;

/*!
 * @brief Classe pour l'estimation de la position des marqueurs
 * @details Estimation(markerLen)
 */
class Estimation {
public:
    explicit Estimation(float markerLen, const char* camera_param_path);

    void lock(const callback_t& callback);
    [[noreturn]] uint8_t start();
private:
    Camera camera;

    float markerLen;
    cv::Vec3d origin;
    cv::Mat image, cameraMatrix, distCoeffs;

    std::mutex mtx;
    std::vector<int> ids;
    std::vector<cv::Vec3d> rVecs, tVecs;
    std::vector<std::vector<cv::Point2f>> corners;
};


#endif //OPENCV_ESTIMATION_H

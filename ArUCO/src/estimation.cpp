//
// Created by mrspaar on 4/25/23.
//

#include <opencv2/aruco.hpp>
#include "estimation.h"


Estimation::Estimation(const cv::FileStorage& configFile): camera(configFile) {
    int status = xbee.openSerialConnection(
            configFile["xbee_port"].string().c_str(),
            (int) configFile["xbee_address"] + 3
    );

    if (status != XB_SER_E_SUCCESS)
        exit(status);

    markerLen = (float) configFile["marker_length_m"];
    refMarkerId = (int) configFile["ref_marker_id"];

    cv::FileStorage fs(configFile["camera_params_path"], cv::FileStorage::READ);

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    fs.release();
}


void Estimation::update() {
    camera.cap >> image;
    camera.detector.detectMarkers(image, corners, ids);

    if (ids.empty())
        return;

    mtx.lock();
    cv::aruco::estimatePoseSingleMarkers(
            corners, markerLen,
            cameraMatrix, distCoeffs,
            rVecs, tVecs
    );

    for(int i=0; i < ids.size(); i++)
        if (ids[i] == refMarkerId) {
            origin = tVecs[i];
            break;
        }

    for (int i=0; i < ids.size(); i++)
        for (int j = 0; j < 3; j++)
            tVecs[i][j] -= origin[j];

    mtx.unlock();
}


void Estimation::send(uint8_t dest) {
    mtx.lock();

    uint16_t temp;
    std::vector<uint8_t> data;

    for (int i=0; i < ids.size(); i++) {
        data.push_back(ids[i]);

        for (int j=0; j < 3; j++) {
            temp = (uint16_t) (tVecs[i][j] * 1000);
            data.push_back((uint8_t) (temp >> 8));
            data.push_back((uint8_t) temp);
        }
    }

    xbee.sendFrame(dest, XB_FCT_ARUCO_POS, data, ids.size()*4);
    mtx.unlock();
}

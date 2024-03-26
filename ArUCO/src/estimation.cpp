//
// Created by mrspaar on 4/25/23.
//

#include <opencv2/aruco.hpp>
#include "estimation.h"


Estimation::Estimation(const cv::FileStorage& configFile): camera(configFile) {
    markerLen = (float) configFile["marker_length_m"];
    refMarkerId = (int) configFile["ref_marker_id"];
    configFile["ref_marker_pos"] >> refMarkerPos;

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

//Bloquage pour eviter de reecrire sur des données qui sont en train d'etre envoyées
    mtx.lock();
    cv::aruco::estimatePoseSingleMarkers(
            corners, markerLen,
            cameraMatrix, distCoeffs,
            rVecs, tVecs
    );

    for(int i=0; i < ids.size(); i++)
        if (ids[i] == refMarkerId) {
            origint = tVecs[i];
            originr = rVecs[i];
            break;
        }

    for (int i=0; i < ids.size(); i++){
        for (int j = 0; j < 3; j++){
            tVecs[i][j] += -origint[j] + refMarkerPos[j];
            std::cout<<"tVecs[" << i << "]"<<"["<< j <<"] = "<< tVecs[i][j] << std::endl;
    }
    rVecs[i][2] -= originr[2];
    std::cout << "rVecs[" << i << "][2] =" << rVecs[i][2] << std::endl;
    }
    mtx.unlock();
}


void Estimation::send(XBee& xbee, uint8_t dest) {
    mtx.lock();

    uint16_t temp;
    std::vector<uint8_t> data;

    data.push_back(refMarkerId);

    for (int i=0; i < ids.size(); i++) {  
        data.push_back(ids[i]);

        for (int j=0; j < 3; j++) {
            temp = (uint16_t) (tVecs[i][j] * 1000);
            data.push_back((uint8_t) (temp >> 8));
            data.push_back((uint8_t) temp);
        }

        temp = (uint16_t) (rVecs[i][2]*1000);
        data.push_back((uint8_t) (temp >> 8));
        data.push_back((uint8_t) temp);
    }

    xbee.send(dest, XB_FCT_ARUCO_POS, data, ids.size()*4);
    mtx.unlock();
}


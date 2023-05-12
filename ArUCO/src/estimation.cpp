//
// Created by mrspaar on 4/25/23.
//

#include "estimation.h"


Estimation::Estimation(const char* detectorParamsPath, const char* cameraParamPath, float markerLen):
    camera(detectorParamsPath), markerLen(markerLen)
{
    cv::FileStorage fs(cameraParamPath, cv::FileStorage::READ);

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    fs.release();
}


[[noreturn]] void Estimation::start() {
    cv::Mat imageCopy;

    while (true) {
        camera.cap >> image;
        camera.detector.detectMarkers(image, corners, ids);

        if (!ids.empty()) {
            mtx.lock();
            cv::aruco::estimatePoseSingleMarkers(
                    corners, markerLen,
                    cameraMatrix, distCoeffs,
                    rVecs, tVecs
            );

            for(int i=0; i < ids.size(); i++)
                if (ids[i] == REF_ARUCO_ID) {
                    origin = tVecs[i];
                    break;
                }

            for (int i=0; i < ids.size(); i++)
                for (int j = 0; j < 3; j++)
                    tVecs[i][j] -= origin[j];

            for (int i=0; i < ids.size(); i++)
                std::cout << ids[i] << ": Rx=" << rVecs[i][0] << " Ry=" << rVecs[i][1] << " Rz=" << rVecs[i][2]
                          << " Tx=" << tVecs[i][0] << " Ty=" << tVecs[i][1] << " Tz=" << tVecs[i][2] << std::endl;

            mtx.unlock();
        }
    }
}


void Estimation::sendTags() {
    mtx.lock();



    mtx.unlock();
}

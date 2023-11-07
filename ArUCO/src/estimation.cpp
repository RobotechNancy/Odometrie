//
// Created by mrspaar on 4/25/23.
//

#include <opencv2/aruco.hpp>
#include "estimation.h"


Estimation::Estimation(const cv::FileStorage& configFile): camera(configFile) {
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

    // Bloquage pour éviter de réécrire sur des données qui sont en train d'être envoyées
    std::lock_guard<std::mutex> lock(mtx);

    cv::aruco::estimatePoseSingleMarkers(
            corners, markerLen,
            cameraMatrix, distCoeffs,
            rVecs, tVecs
    );

    // On récupère la position du marqueur de référence
    for(int i=0; i < ids.size(); i++)
        if (ids[i] == refMarkerId) {
            origin = tVecs[i];
            break;
        }

    // On soustrait la position du marqueur de référence à toutes les autres
    for (int i=0; i < ids.size(); i++)
        for (int j = 0; j < 3; j++)
            tVecs[i][j] -= origin[j];
}


void Estimation::send(XBee& xbee, uint8_t dest) {
    uint16_t temp;
    std::vector<uint8_t> data;

    {
        // On bloque jusqu'à ce qu'on ait récupéré les données
        std::lock_guard<std::mutex> lock(mtx);

        // On envoie id, Tx, Ty et Tz pour chaque marqueur
        for (int i = 0; i < ids.size(); i++) {
            data.push_back(ids[i]);

            for (int j = 0; j < 3; j++) {
                temp = (uint16_t) (tVecs[i][j] * 1000); // Conversion en mm
                data.push_back((uint8_t) (temp >> 8));  // ⎡ On ne peut qu'envoyer octets par octets
                data.push_back((uint8_t) temp);         // ⎣ Donc on sépare les 16 bits en 2 octets
            }
        }
    }

    xbee.send(dest, XB_FCT_ARUCO_POS, data);
}

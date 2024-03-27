//
// Created by mrspaar on 4/25/23.
// Modified by garatim & Manal
//

#include <opencv2/aruco.hpp>
#include "estimation.h"


Estimation::Estimation(const cv::FileStorage& configFile, int refMid, cv::Vec3d refMarkerPos): camera(configFile) {
    markerLen = (float) configFile["marker_length_m"];
    //refMarkerId = (int) configFile["ref_marker_id"];
    //configFile["ref_marker_pos"] >> refMarkerPos;

    refMarkerId=refMid;

    cv::FileStorage fs(configFile["camera_params_path"], cv::FileStorage::READ);

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    fs.release();
}


//Fontion de test du bon positionnement de la caméra
bool Estimation::cameraPosition() {

    camera.cap >> image;
    camera.detector.detectMarkers(image, corners, ids);

    if (ids.empty()){
        std::cout<<"No code detected"<<std::endl;}

    mtx.lock();

    for(int i=0; i < ids.size(); i++){
	std::cout << ids[i] << std::endl;
        if (ids[i] == refMarkerId) {
            return true;
        }
    }

    return false;
}


//Fonction de mise à jour de l'estimation des positions des codes ArUCO détectés
void Estimation::update() {
    //capture de l'image et récupération des infos
    camera.cap >> image;
    camera.detector.detectMarkers(image, corners, ids);

    //arrêt si aucun code détecté
    if (ids.empty())
        return;

    //Bloquage pour eviter de reecrire sur des données qui sont en train d'etre envoyées
    mtx.lock();
    //Estimation de la position des marqueurs (relatif à l'image)
    //tVecs = tableau des positions des codes [x, y, z]
    //rVecs =  tableau des rotations des codes [o, Tz, o]
    cv::aruco::estimatePoseSingleMarkers(
            corners, markerLen,
            cameraMatrix, distCoeffs,
            rVecs, tVecs
    );

    //Définition de la position et de la rotation du code repère comme origines
    for(int i=0; i < ids.size(); i++)
        if (ids[i] == refMarkerId) {
            origint = tVecs[i];
            originr = rVecs[i];
            break;
        }

    //Double changements de repère pour mettre la position des codes par rapport au 0/0 du plateau 
    for (int i=0; i < ids.size(); i++){
        for (int j = 0; j < 3; j++){
            tVecs[i][j] += -origint[j] + refMarkerPos[j];
            //Affichage des veleurs obtenus pour vérification lors des tests/débeugage
            std::cout<<"tVecs[" << i << "]"<<"["<< j <<"] = "<< tVecs[i][j] << std::endl;
    }
    rVecs[i][2] -= originr[2];
    std::cout << "rVecs[" << i << "][2] =" << rVecs[i][2] << std::endl;
    }
    mtx.unlock();
}

//Fonction d'envoie des données de l'estimation
void Estimation::send(XBee& xbee, uint8_t dest) {
    mtx.lock();

    uint16_t temp;
    std::vector<uint8_t> data;

    //Pousse l'ID du marqueur de ref dans les données
    data.push_back(refMarkerId);

    for (int i=0; i < ids.size(); i++) {  
        //Pousse l'ID du code dans les données
        data.push_back(ids[i]);

        for (int j=0; j < 3; j++) {
            //Converion de m à mm
            temp = (uint16_t) (tVecs[i][j] * 1000);
            //Pousse la coordonnée dans les données (séparé en deux octets pour être assez grand)
            data.push_back((uint8_t) (temp >> 8));
            data.push_back((uint8_t) temp);
        }

        temp = (uint16_t) (rVecs[i][2]*1000);
        //Pousse l'orientation dans les données (séparé en deux octets pour être assez grand)
        data.push_back((uint8_t) (temp >> 8));
        data.push_back((uint8_t) temp);
    }

    //Envoie des données et le nombre de codes repérés
    xbee.send(dest, XB_FCT_ARUCO_POS, data, ids.size()*4);
    mtx.unlock();
}


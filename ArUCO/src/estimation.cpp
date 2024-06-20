//
// Created by mrspaar on 4/25/23.
// Modified by garatim & Manal
//


#include <iostream>
#include <opencv2/opencv.hpp> // Include OpenCV headers
#include <opencv2/aruco.hpp>
#include "estimation.h"
#include "calculsfct.h"




//Constructeur(/Protoype) de l'objet Estimation
Estimation::Estimation(const cv::FileStorage& configFile, int refMid, cv::Vec3d refMpos): camera(configFile) {
    markerLen = (float) configFile["marker_length_m"];  //récupération de la longueur d'un marqueur depuis le fichier de configuration

    refMarkerId= refMid;                                //récupération de l'id du marqueur de référence
    refMarkerPos = refMpos;                             //récupération de la position (coords) du marqueur de référence

    std::vector<cv::Vec3d> rAngles;                     //initialisation de la variable rAngles (angles de rotations euler des codes ArUCO)
    std::vector<cv::Vec3d> transMat;                    //initialisation de la variable transMat (matrice de translation obtenu après tranformation pour avoir les coords des codes ArUCO)
    cv::FileStorage fs(configFile["camera_params_path"], cv::FileStorage::READ);    //lecture du fichier pour récupérer les paramètres de la caméra

    fs["camera_matrix"] >> cameraMatrix;                //récupération de la matrice de la caméra
    fs["distortion_coefficients"] >> distCoeffs;        //récupération des coefficients de distortion de la caméra
    fs.release();           //libère les ressources associées et ferme le fichier de config (libère toute mémoire allouée pour le stockage des données du fichier)
}




//Ecriture des mesures dans un fichier :
int ecrituremesures(std::vector<int> ids, std::vector<cv::Vec3d> tVecs) {
    // récupère la date et l'heure actuelles
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);
    std::string datetime(buf);
    // génère un fichier texte avec un nom contenant la date-heure et l'ouvre pour l'écriture
    std::string filename = "estimCaptures/capture-" + datetime + ".txt";
    std::ofstream outFile(filename);
    // vérifie s'il est bien ouvert
    if (!outFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
    }
    // écrit l'en-tête
    outFile << "IDs\tX\tY\tZ" << std::endl;
    // écrit les donées
    for (size_t i = 0; i < ids.size(); ++i) {
        outFile << ids[i] << "\t" << tVecs[i][0] << "\t" << tVecs[i][1] << "\t" << tVecs[i][2] << "\n" << std::endl;
    }
    // ferme le fichier
    outFile.close();
    return 0;
}




//Fonction de mise à jour de l'estimation des positions des codes ArUCO détectés
void Estimation::update() {
    //capture de l'image et récupération des infos
    camera.cap >> image;
    camera.detector.detectMarkers(image, corners, ids);

    //arrêt si aucun code détecté
    if (ids.empty())
        return;

    //bloquage pour eviter de réécrire sur des données qui sont en train d'etre envoyées
    mtx.lock();


    //Estimation de la position des marqueurs (relatif à l'image)
    // tVecs = vecteurs de translation des codes [x, y, z] (repère caméra)
    // rVecs =  vecteurs de rotation des codes [o, a, t] 
    cv::aruco::estimatePoseSingleMarkers(
            corners, markerLen,
            cameraMatrix, distCoeffs,
            rVecs, tVecs
    );




    //Affichage des valeurs de tVecs et rVecs de chaque code issue de la fonction d'estimation d'openCV
    for (int i=0; i < ids.size(); i++){
	    std::cout << std::dec << "-Code : " << ids[i] << std::endl;
        std::cout << "  Vecteur de translation (tVecs) : " << std::endl;
        for (int j = 0; j < 3; j++){
            std::cout << "      tVecs[" << i << "][" << j << "] = " << tVecs[i][j] << std::endl;
	    }
        std::cout << "  Vecteur de rotation (rVecs) : " << std::endl;
        for (int j = 0; j < 3; j++){
            std::cout << "      rVecs[" << i << "][" << j << "] = " << rVecs[i][j] << std::endl;
	    }
    }



    //Récupération de l'indice du marqueur de référence
    for(int i=0; i < ids.size(); i++) {
        if (ids[i] == refMarkerId) {
            refMarkerIndex = i;
        }
    }



    //Converion du vecteur de rotation pour obtenir la rotation des codes
    rAngles = RotationTransform(rVecs);



    //Transformation des données des codes ArUCO pour obtenir leurs coordonnées relative au code de référence
    transMat = PositionTransform_1(ids, rVecs, tVecs, refMarkerIndex);
    

    //Si erreur ou non fonctionnement de la transformation, utilisation de tVecs :
    //transMat = tVecs;



    //Définition de la position et de la rotation du code repère comme origines
    origint = transMat[refMarkerIndex];
    originr = rAngles[refMarkerIndex];
    


    //Affichage des valeurs de transMat et rAngles de chaque code issue des transformations
    //+ changements de repère pour mettre la position des codes par rapport au 0/0 du plateau
    std::cout << " " << std::endl; 
    for (int i=0; i < ids.size(); i++){
	    std::cout << std::dec << "-Code : " << ids[i] << std::endl;
        std::cout << "  Marice de position (transMat) : " << std::endl;
        for (int j = 0; j < 3; j++){
            std::cout << "      transMat(relative)[" << i << "][" << j << "] = " << transMat[i][j] << std::endl;
            transMat[i][j] +=  refMarkerPos[j]; // -origint[j] + 
            //std::cout << "      transMat(absolue)[" << i << "][" << j << "] = " << transMat[i][j] << std::endl;
	    }
        std::cout << "  Rotations (rAngles) : " << std::endl;
        for (int j = 0; j < 3; j++){
            std::cout << "      rAngles(relative)[" << i << "][" << j << "] = " << rAngles[i][j] << std::endl;
            rAngles[i][j] -= originr[j];
            //std::cout << "      rAngles(absolue)[" << i << "][" << j << "] = " << rAngles[i][j] << std::endl;
	    }
    }

    

    //Ecriture des mesures dans un fichier :
    //int e = ecrituremesures(ids, tVecs);



    mtx.unlock();
}





//Fonction d'envoie des données de l'estimation
void Estimation::send(XBee& xbee, uint8_t dest) {
    mtx.lock();

    //initialisation des variables temporaires et de données
    uint16_t temp;
    std::vector<uint8_t> data;

    //pousse l'ID du marqueur de ref dans les données
    data.push_back(refMarkerId);

    for (int i=0; i < ids.size(); i++) {  
        //pousse l'ID du code dans les données
        data.push_back(ids[i]);

        for (int j=0; j < 3; j++) {
            temp = (uint16_t) (transMat[i][j]);
            //pousse la coordonnée dans les données (séparé en deux octets pour être assez grand)
            data.push_back((uint8_t) (temp >> 8));
            data.push_back((uint8_t) temp);
        }
        //repère à +180 pour être positif et avec un facteur 10 pour pouvoir envoyer une précision à 0,1°
        temp = (uint16_t) ((rAngles[i][2]+180)*10);
        //pousse l'orientation dans les données (séparé en deux octets pour être assez grand)
        data.push_back((uint8_t) (temp >> 8));
        data.push_back((uint8_t) temp);
    }

    //Envoie des données et le nombre de codes repérés
    xbee.send(dest, XB_FCT_ARUCO_POS, data, ids.size()*4);
    mtx.unlock();
}


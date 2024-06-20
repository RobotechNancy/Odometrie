//
// Created by garatim on 09/05/24.
//


#include <iostream>
#include <opencv2/opencv.hpp> // Include OpenCV headers
#include <opencv2/aruco.hpp>
#include <unistd.h>
#include "tests.h"


//Constructeur(/Protoype) de l'objet Tests
Tests::Tests(const cv::FileStorage& configFile, int refMid, cv::Vec3d refMpos): camera(configFile) {
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





// Vérifie la distance entre les coins
int Tests::verifCoherenceArUCO(const std::vector<std::vector<cv::Point2f>>& corners, int i) {
    float distance;
    
    distance = cv::norm(corners[i][0] - corners[i][1]); // Distance entre coin 0 et coin 1
    if (std::abs(distance - Dx) < 10) // Si la distance n'est pas proche de la valeur attendue
        return 1;

    distance = cv::norm(corners[i][1] - corners[i][2]); // Distance entre coin 1 et coin 2
    if (std::abs(distance - Dy) < 10)
        return 2;

    distance = cv::norm(corners[i][2] - corners[i][3]); // Distance entre coin 2 et coin 3
    if (std::abs(distance - Dx) < 10)
        return 3;

    distance = cv::norm(corners[i][3] - corners[i][0]); // Distance entre coin 3 et coin 0
    if (std::abs(distance - Dy) < 10)
        return 4;

    return 0; // Tous les coins sont cohérents
}



// Test de la cohérence des marqueurs ArUCO détéctés
int Tests::testCoherence() {

    cv::Mat image;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    while (true) {
        camera.cap >> image;
        camera.detector.detectMarkers(image, corners, ids);

        if (!ids.empty()) {
            for (int i = 0; i < ids.size(); i++) {
                int consistency = verifCoherenceArUCO(corners, i);
                std::cout << "Marker ID: " << ids[i] << std::endl;
                if (consistency == 0) {
                    std::cout << "  Les 4 coins sont cohérents." << std::endl;
                } else
                    std::cout << "  Le coin " << consistency << " n'est pas cohérent." << std::endl;
                std::cout << "    Coordinates 1 : (" << corners[i][0].x << ", " << corners[i][0].y << "), Coordinates 2 : (" << corners[i][1].x << ", " << corners[i][1].y << "), Coordinates 3 : (" << corners[i][2].x << ", " << corners[i][2].y << "), Coordinates 4 : (" << corners[i][3].x << ", " << corners[i][3].y << ")" << std::endl;
            }
        }
        std::cout << "  " << std::endl;

        cv::imshow("ArUCO", image);

        char key = cv::waitKey(1);
        if (key == 27) //touche 'ESC' pour quitter
            break;

        sleep(2);
    }

    return 0;
}




//Fontion de test du bon positionnement de la caméra
bool Tests::cameraPosition() {
    camera.cap >> image;
    camera.detector.detectMarkers(image, corners, ids);

    if (ids.empty()){
        std::cout<<"No code detected"<<std::endl;}

    mtx.lock();

    for(int i=0; i < ids.size(); i++){
        if (ids[i] == refMarkerId) {
	    std::cout << ids[i] << std::endl;
            return true;
        }
    }

    return false;
}




//Fontion de test de la détection
int Tests::testDetection() {
    cv::Mat img, imgCopy;
    while (true) {
        // Capture frame from camera
        camera.cap >> img;
        img.copyTo(imgCopy);
        // Detect markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        // detect markers and estimate pose
        camera.detector.detectMarkers(imgCopy, corners, ids);
        size_t nMarkers = corners.size();
        std::vector<cv::Vec3d> rvec, tvec;
        // Calculate pose for each marker
        cv::aruco::estimatePoseSingleMarkers(corners, markerLen, cameraMatrix, distCoeffs, rvec, tvec);
        // Draw detected markers and their IDs
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(imgCopy, corners, ids);
            for(unsigned int i = 0; i < ids.size(); i++) {
                cv::drawFrameAxes(imgCopy, cameraMatrix, distCoeffs, rvec[i], tvec[i], markerLen * 1.5f, 2);
            }
        }
        // Display the frame
        cv::imshow("ArUCO Detection", imgCopy);
        // Check for ESC key press
        if (cv::waitKey(30) == 27) {
            break;
        }
    }
    // Release the camera
    camera.cap.release();
    cv::destroyAllWindows();
    return 0;
}




//Fontion de test de la position
int Tests::testPosition() {
    cv::Mat img, imgCopy;
    while (true) {
        // Capture frame from camera
        camera.cap >> img;
        img.copyTo(imgCopy);
        // Detect markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        camera.detector.detectMarkers(imgCopy, corners, ids);
        size_t nMarkers = corners.size();
        // Calculate pose for each marker
        // Draw detected markers and their IDs
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(imgCopy, corners, ids);
            for (size_t i = 0; i < ids.size(); ++i) {
                std::cout << "Marker ID: " << ids[i] << ", Coordinates: (" << corners[i][0].x << ", " << corners[i][0].y << ")" << std::endl;
            }
        }
        std::cout << " " << std::endl;
        // Check for ESC key press
        if (cv::waitKey(30) == 27) {
            break;
        }
        sleep(2);
    }
    // Release the camera
    camera.cap.release();
    return 0;
}
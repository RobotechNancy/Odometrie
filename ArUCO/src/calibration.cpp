//
// Created by mrspaar on 4/25/23.
//

#include <iostream>

#include "camera.h"
#include "calibration.h"
#include "opencv2/aruco.hpp"
#include <opencv2/highgui.hpp>


void boardToPng(const cv::FileStorage& configFile) {
    // On récupère les paramètres de la grille
    int markersX = (int) configFile["markers_x"];
    int markersY = (int) configFile["markers_y"];
    int markerLen = (int) configFile["marker_length_px"];
    int markerSep = (int) configFile["markers_spacing_px"];
    std::string boardSavePath = (std::string) configFile["board_save_path"];

    int margins = markerSep;
    int borderBits = 1;

    cv::Size imageSize;
    imageSize.width = markersX * (markerLen + markerLen) - markerSep + 2 * margins;
    imageSize.height = markersY * (markerLen + markerSep) - markerSep + 2 * margins;

    // On génère la grille à partir des paramètres et du dictionnaire choisi
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary((int) configFile["dictionary"]);
    cv::aruco::GridBoard b(cv::Size(markersX, markersY), float(markerLen), float(markerSep), dictionary);

    cv::Mat boardImage;
    b.generateImage(imageSize, boardImage, margins, borderBits);

    // On montre la grille et on attend une touche pour la sauvegarder
    cv::imshow("board", boardImage);
    cv::waitKey(0);

    imwrite(boardSavePath, boardImage);
    std::cout << "Grille sauvegardée dans \"" << boardSavePath << "\"" << std::endl;
}


// Script adapté des tutoriels OpenCV
uint8_t calibrate(const cv::FileStorage& configFile) {
    Camera camera(configFile);

    cv::Size imgSize;
    cv::Mat img, imgCopy; // On utilise img pour les calculs et imgCopy pour l'affichage avec réalité augmentée

    std::vector<int> calib_ids;
    std::vector<std::vector<int>> allIds;

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;

    // Boucle pour récupérer les données de calibration
    while (true) {
        camera.cap >> img;
        img.copyTo(imgCopy);

        camera.detector.detectMarkers(imgCopy, corners, calib_ids);
        if (!calib_ids.empty())
            cv::aruco::drawDetectedMarkers(imgCopy, corners, calib_ids);

        imshow("out", imgCopy);
        char key = (char) cv::waitKey(10);

        // On quitte si on appuie sur 'esc'
        if (key == 27)
            break;

        // On mémorise les données de calibration si on appuie sur 'c'
        if (key == 'c' && !calib_ids.empty()) {
            std::cout << "Image capturée" << std::endl;
            allCorners.push_back(corners);
            allIds.push_back(calib_ids);
            imgSize = img.size();
        } else if (key == 'c') {
            std::cout << "Aucun marqueur détecté" << std::endl;
        }

        calib_ids.clear();
        corners.clear();
    }

    camera.cap.release();

    if (allIds.empty()) {
        std::cerr << "Pas assez de captures pour calibrer la caméra" << std::endl;
        return -1;
    }

    // Paramètres de caméra (vides pour l'instant)
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> calib_rvecs, calib_tvecs;
    double repError;

    std::vector<std::vector<cv::Point2f> > allCornersConcatenated;
    std::vector<int> allIdsConcatenated;
    std::vector<int> markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());

    // On concatène les vecteurs de vecteurs de points et d'ids
    for (unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int) allCorners[i].size());

        for (unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    // On recrée la grille à partir des paramètres et du dictionnaire choisi
    cv::aruco::GridBoard gridBoard(
            cv::Size(
                    (int) configFile["markers_x"],
                    (int) configFile["markers_y"]
            ),
            (float) configFile["marker_length_m"],
            (float) configFile["markers_spacing_m"],
            camera.detector.getDictionary()
    );

    // On calcule les paramètres de caméra à partir des captures et de la grille recréée
    repError = calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                    markerCounterPerFrame, &gridBoard, imgSize, cameraMatrix,
                                    distCoeffs, calib_rvecs, calib_tvecs, 0);

    // On enregistre les paramètres de caméra dans un fichier YAML
    std::string cameraParamsPath = (std::string) configFile["camera_params_path"];
    cv::FileStorage fs(cameraParamsPath, cv::FileStorage::WRITE);

    if (!fs.isOpened()) {
        std::cerr << "Impossible de créer le fichier " << cameraParamsPath << std::endl;
        return -1;
    }

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;
    fs << "image_width" << imgSize.width;
    fs << "image_height" << imgSize.height;
    fs << "aspectRatio" << 1;
    fs << "flags" << 0;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "avg_reprojection_error" << repError;

    std::cout << "Paramètres de caméra enregistrés dans " << cameraParamsPath << std::endl;

    return 0;
}

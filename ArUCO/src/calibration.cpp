//
// Created by mrspaar on 4/25/23.
// modified by garatim
//

#include <iostream>

#include "camera.h"
#include "calibration.h"
#include "opencv2/aruco.hpp"
#include <opencv2/highgui.hpp>



//Fonction génèrant une grille de marqueurs ArUco à partir des paramètres stockés dans un fichier de configuration,
//affiche la grille à l'écran et sauvegarde l'image résultante au format PNG
void boardToPng(const cv::FileStorage& configFile) {
    // On récupère les paramètres de la grille
    int markersX = (int) configFile["markers_x"];
    int markersY = (int) configFile["markers_y"];
    int markerLen = (int) configFile["marker_length_px"];
    int markerSep = (int) configFile["markers_spacing_px"];
    std::string boardSavePath = (std::string) configFile["board_save_path"];

    // Calcul des marges et des bits de bordure pour la génération de l'image
    int margins = markerSep;
    int borderBits = 1;

    // Calcul de la taille de l'image en fonction des paramètres de la grille
    cv::Size imageSize;
    imageSize.width = markersX * (markerLen + markerLen) - markerSep + 2 * margins;
    imageSize.height = markersY * (markerLen + markerSep) - markerSep + 2 * margins;

    // On génère la grille à partir des paramètres et du dictionnaire choisi
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary((int) configFile["dictionary"]);
    cv::aruco::GridBoard b(cv::Size(markersX, markersY), float(markerLen), float(markerSep), dictionary);

    // Génération de l'image de la grille
    cv::Mat boardImage;
    b.generateImage(imageSize, boardImage, margins, borderBits);

    // On montre la grille et on attend une touche pour la sauvegarder
    cv::imshow("board", boardImage);
    cv::waitKey(0);

    // Sauvegarde de l'image de la grille au format PNG
    imwrite(boardSavePath, boardImage);
    std::cout << "Grille sauvegardée dans \"" << boardSavePath << "\"" << std::endl;
}




// (Script adapté des tutoriels OpenCV)
// Cette fonction effectue la calibration de la caméra en utilisant des marqueurs ArUco détectés dans les images capturées.
// Elle collecte les données de calibration à partir des images capturées et sauvegarde les paramètres de caméra calculés dans un fichier YAML.
uint8_t calibrate(const cv::FileStorage& configFile) {
    Camera camera(configFile);

    // Initialisation des variables pour stocker les données de calibration
    cv::Size imgSize;
    cv::Mat img, imgCopy; // On utilise img pour les calculs et imgCopy pour l'affichage avec réalité augmentée
    std::vector<int> calib_ids;
    std::vector<std::vector<int>> allIds;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;

    // Boucle pour récupérer les données de calibration
    while (true) {
        // Capture d'une image à partir de la caméra
        camera.cap >> img;
        img.copyTo(imgCopy);

        // Détection des marqueurs ArUco dans l'image capturée
        camera.detector.detectMarkers(imgCopy, corners, calib_ids);
        // Dessin des marqueurs détectés sur l'image
        if (!calib_ids.empty())
            cv::aruco::drawDetectedMarkers(imgCopy, corners, calib_ids);

        // Affichage de l'image avec les marqueurs détectés
        imshow("out", imgCopy);

        // On quitte si on appuie sur 'esc'
        char key = (char) cv::waitKey(10);
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

        // Réinitialisation des vecteurs pour la prochaine itération
        calib_ids.clear();
        corners.clear();
    }

    // Libération des ressources de la caméra
    camera.cap.release();

    // Vérification s'il y a suffisamment de données de calibration pour procéder
    if (allIds.empty()) {
        std::cerr << "Pas assez de captures pour calibrer la caméra" << std::endl;
        return -1;
    }

    // Paramètres de caméra (vides pour l'instant)
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> calib_rvecs, calib_tvecs;
    double repError;
    // Concaténation des données de calibration pour les utiliser dans la fonction de calibration
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
    cv::Ptr<cv::aruco::GridBoard> gridBoard = new cv::aruco::GridBoard(
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
                                    markerCounterPerFrame, gridBoard, imgSize, cameraMatrix,
                                    distCoeffs, calib_rvecs, calib_tvecs, 0);

    // On enregistre les paramètres de caméra dans un fichier YAML
    std::string cameraParamsPath = (std::string) configFile["camera_params_path"];
    cv::FileStorage fs(cameraParamsPath, cv::FileStorage::WRITE);

    // Vérification de l'ouverture réussie du fichier de sortie
    if (!fs.isOpened()) {
        std::cerr << "Impossible de créer le fichier " << cameraParamsPath << std::endl;
        return -1;
    }


    // Enregistrement des paramètres de caméra dans le fichier YAML
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

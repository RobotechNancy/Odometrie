//
// Created by mrspaar on 4/25/23.
//

#include <iostream>

#include "camera.h"
#include "calibration.h"
#include "opencv2/aruco.hpp"
#include <opencv2/highgui.hpp>


void boardToPng(const cv::FileStorage& configFile) {
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

    cv::aruco::Dictionary dictionary = getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::GridBoard b(cv::Size(markersX, markersY), float(markerLen), float(markerSep), dictionary);

    cv::Mat boardImage;
    b.generateImage(imageSize, boardImage, margins, borderBits);

    cv::imshow("board", boardImage);
    cv::waitKey(0);

    imwrite(boardSavePath, boardImage);
    std::cout << "Grille sauvegardée dans \"" << boardSavePath << "\"" << std::endl;
}


uint8_t calibrate(const cv::FileStorage& configFile) {
    Camera camera(configFile);

    cv::Size imgSize;
    cv::Mat img, imgCopy;

    std::vector<int> calib_ids;
    std::vector<std::vector<int>> allIds;

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;

    while (true) {
        camera.cap >> img;
        img.copyTo(imgCopy);

        camera.detector.detectMarkers(imgCopy, corners, calib_ids);
        if (!calib_ids.empty())
            cv::aruco::drawDetectedMarkers(imgCopy, corners, calib_ids);

        imshow("out", imgCopy);
        char key = (char) cv::waitKey(10);

        if (key == 27)
            break;

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

    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> calib_rvecs, calib_tvecs;
    double repError;

    std::vector<std::vector<cv::Point2f> > allCornersConcatenated;
    std::vector<int> allIdsConcatenated;
    std::vector<int> markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());

    for (unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int) allCorners[i].size());

        for (unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    cv::aruco::GridBoard gridBoard(
            cv::Size(
                    (int) configFile["markers_x"],
                    (int) configFile["markers_y"]
            ),
            (float) configFile["marker_length_m"],
            (float) configFile["markers_spacing_m"],
            camera.detector.getDictionary()
    );

    repError = calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                    markerCounterPerFrame, &gridBoard, imgSize, cameraMatrix,
                                    distCoeffs, calib_rvecs, calib_tvecs, 0);

    cv::FileStorage fs("../camera_params.yml", cv::FileStorage::WRITE);

    if (!fs.isOpened()) {
        std::cerr << "Impossible de créer le fichier \"camera_params.yml\"" << std::endl;
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

    std::cout << "Paramètres de caméra enregistrés dans \"camera_params.yml\"" << std::endl;

    return 0;
}

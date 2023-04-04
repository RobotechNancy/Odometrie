//
// Created by mrspaar on 1/17/23.
//

#include "../include/estimator.h"

using namespace cv;
using namespace std;
using namespace aruco;


void writeOnImage(Mat &img, const string &text, const Point &position) {
    putText(img, text, position, FONT_HERSHEY_PLAIN, 1, Scalar::all(0), 3);
    putText(img, text, position, FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1);
}


Estimator::Estimator() {
    /*int status = xbee.openSerialConnection();

    if(status != XB_SER_E_SUCCESS)
        exit(status);

    thread heartbeat(&XBee::sendHeartbeat, &xbee);
    thread waitingtrame(&XBee::waitForATrame, &xbee);
    thread reponse(&XBee::isXbeeResponding, &xbee);*/

    origin = Vec3d(0, 0, 0);
    cap = *new VideoCapture(0, CAP_V4L2);

    if (!cap.isOpened()) {
        cerr << "Failed to open video input " << endl;
        exit(-1);
    }

    FileStorage fs("../detector_params.yml", FileStorage::READ);

    if (!fs.isOpened()) {
        cerr << "Failed to open detector parameters file" << endl;
        exit(-1);
    }

    DetectorParameters detectorParams;
    detectorParams.readDetectorParameters(fs.root());
    fs.release();

    detector.setDetectorParameters(detectorParams);
    detector.setDictionary(getPredefinedDictionary(DICT_4X4_100));
}


int Estimator::calibrate(int markersX, int markersY, float markerLength, float markerSeparation) {
    GridBoard board(
            Size(markersX, markersY),
            float(markerLength),
            float(markerSeparation),
            detector.getDictionary()
    );

    Size imgSize;
    Mat img, imgCopy;

    vector<int> calib_ids;
    vector<vector<int>> allIds;

    vector<vector<Point2f>> corners;
    vector<vector<vector<Point2f>>> allCorners;

    while (true) {
        cap >> img;
        img.copyTo(imgCopy);
        detector.detectMarkers(imgCopy, corners, calib_ids);

        if (!calib_ids.empty())
            drawDetectedMarkers(imgCopy, corners, calib_ids);

        imshow("out", imgCopy);
        char key = (char) waitKey(10);

        if (key == 27)
            break;

        if (key == 'c' && !calib_ids.empty()) {
            cout << "Frame captured" << endl;
            allCorners.push_back(corners);
            allIds.push_back(calib_ids);
            imgSize = img.size();
        }

        calib_ids.clear();
        corners.clear();
    }

    cap.release();

    if (allIds.empty()) {
        cerr << "Not enough captures for calibration" << endl;
        return -1;
    }

    Mat cameraMatrix, distCoeffs;
    vector<Mat> calib_rvecs, calib_tvecs;
    double repError;

    vector<vector<Point2f> > allCornersConcatenated;
    vector<int> allIdsConcatenated;
    vector<int> markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());

    for (unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int) allCorners[i].size());

        for (unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    repError = calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                    markerCounterPerFrame, &board, imgSize, cameraMatrix,
                                    distCoeffs, calib_rvecs, calib_tvecs, 0);

    FileStorage fs("../camera_params.yml", FileStorage::WRITE);

    if (!fs.isOpened()) {
        cerr << "Failed to open camera parameters file" << endl;
        exit(-1);
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

    cout << "Calibration saved to camera_params.yml" << endl;

    return 0;
}


int Estimator::start() {
    Mat image, imageCopy;
    Mat cameraMatrix, distCoeffs;
    ostringstream vector2Marker;
    vector<vector<Point2f>> corners;

    FileStorage fs("../camera_params.yml", FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    while (cap.grab()) {
        cap.retrieve(image);
        image.copyTo(imageCopy);

        mtx.lock();
        detector.detectMarkers(image, corners, ids);

        if (!ids.empty()) {
            estimatePoseSingleMarkers(corners, marker_length, cameraMatrix, distCoeffs, rvecs, tvecs);
            drawDetectedMarkers(imageCopy, corners, ids);

            for(int i=0; i < ids.size(); i++)
                if (ids[i] == REF_ARUCO_ID) {
                    origin = tvecs[i];
                    break;
                }

            for (int i=0; i < ids.size(); i++)
                for (int j = 0; j < 3; j++)
                    tvecs[i][j] -= origin[j];

            xbee.sendTrame(XB_ADR_BROADCAST, XB_FCT_ARUCO_POS, to_bytes());
        } else
            writeOnImage(imageCopy, "No markers detected, press 'esc' to exit", Point(10, 30));

        //mtx.unlock();
        imshow("out", imageCopy);

        if (waitKey(IMG_DELAY) == BREAK_KEY)
            break;
    }

    mtx.unlock();
    xbee.closeSerialConnection();
    return 0;
}


char *Estimator::to_bytes() {
    char *bytes = new char[ids.size()*4];
    mtx.lock();

    for (int i=0; i < ids.size(); i++) {
        bytes[i*4] = (char) ids[i];
        bytes[i*4+1] = (char) tvecs[i][0];
        bytes[i*4+2] = (char) tvecs[i][1];
        bytes[i*4+3] = (char) tvecs[i][2];
    }

    mtx.unlock();
    return bytes;
}

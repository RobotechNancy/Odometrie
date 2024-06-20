
// Autre programme présenté à la coupe



#include <iostream>
#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// ------------
// --- ROTATIONS  https://www.learnopencv.com/rotation-matrix-to-euler-angles/
// ------------

bool isRotationMatrix(const cv::Mat &R) {
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
    cv::Mat diff = I - shouldBeIdentity;
    double n = cv::norm(diff);
    return n < 1e-6;
}

cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R) {
    assert(isRotationMatrix(R));
    double sy = std::sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2,1), R.at<double>(2,2));
        y = std::atan2(-R.at<double>(2,0), sy);
        z = std::atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = std::atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = std::atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}

const double MARKER_SIZE = 50; // mm
const int FRAME_RATE = 40;

int main() {
    cv::Mat mtx, dist;
    std::string cal_fname = "camera-cal-cpu.npy";
    std::ifstream file(cal_fname, std::ios::binary);
    file.read(reinterpret_cast<char*>(mtx.data), 9 * sizeof(double));
    file.read(reinterpret_cast<char*>(dist.data), 5 * sizeof(double));
    file.close();

    cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, FRAME_RATE);

    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, aruco_dict);

    cv::Mat frame;
    while (true) {
        cap >> frame;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        cv::aruco::detectMarkers(gray, aruco_dict, corners, ids, parameters);

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            cv::Vec3d rvec, tvec;
            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE, mtx, dist, rvec, tvec);

            cv::aruco::²drawAxis(frame, mtx, dist, rvec, tvec, MARKER_SIZE * 0.5f);

            cv::Vec3d rvec_flipped = -rvec;
            cv::Vec3d tvec_flipped = -tvec;

            cv::Mat rotmat;
            cv::Rodrigues(rvec_flipped, rotmat);
            cv::Vec3f euler_angles = rotationMatrixToEulerAngles(rotmat);

            cv::Mat w_tvec = rotmat * tvec_flipped;

            std::string tvec_str = "x=" + std::to_string(w_tvec.at<double>(0)) +
                                   " y=" + std::to_string(w_tvec.at<double>(1)) +
                                   " z=" + std::to_string(w_tvec.at<double>(2)) +
                                   " direction=" + std::to_string(cv::saturate_cast<int>(std::round(cv::fastAtan2(tvec_flipped[1], tvec_flipped[0]))));
            cv::putText(frame, tvec_str, cv::Point(20, 460), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }

        cv::imshow("Image Feed", frame);

        char pressedKey = cv::waitKey(1);
        if (pressedKey == 'c') {
            static int shot_idx = 0;
            std::string shot_fname = "photos/cpu/detect/" + std::to_string(shot_idx) + ".jpg";
            cv::imwrite(shot_fname, frame);
            shot_idx++;
        } else if (pressedKey == 'q') {
            break;
        }
    }

    return 0;
}

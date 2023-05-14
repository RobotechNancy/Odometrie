//
// Created by mrspaar on 4/25/23.
//

#include "estimation.h"
#include "calibration.h"
#include "robotech/xbee.h"


int main(int argc, char** argv) {
    cv::FileStorage fs("../data/lib_params.yml", cv::FileStorage::READ);

    if (argc > 1 && strcmp(argv[1], "calibrate") == 0) {
        Camera camera(fs);

        return calibrate(
                camera,
                (int) fs["markers_x"],
                (int) fs["markers_y"],
                (float) fs["marker_length_m"],
                (float) fs["markers_spacing_m"]
        );
    }

    if (argc > 1 && strcmp(argv[1], "board") == 0) {
        return boardToPng(
                "../data/board.png",
                (int) fs["markers_x"],
                (int) fs["markers_y"],
                (int) fs["marker_length_px"],
                (int) fs["markers_spacing_px"]
        ), 0;
    }

    if (argc < 2 || strcmp(argv[1], "estimate") != 0) {
        std::cout << "Usage: " << argv[0] << " [calibrate|estimate|board]" << std::endl;
        return -1;
    }

    Estimation estimation(fs);
}

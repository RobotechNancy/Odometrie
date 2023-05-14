//
// Created by mrspaar on 4/25/23.
//

#include "estimation.h"
#include "calibration.h"


int main(int argc, char** argv) {
    cv::FileStorage fs("../data/lib_params.yml", cv::FileStorage::READ);

    if (argc > 1 && strcmp(argv[1], "calibrate") == 0) {
        return calibrate(fs);
    }

    if (argc > 1 && strcmp(argv[1], "board") == 0) {
        return boardToPng(fs), 0;
    }

    if (argc < 2 || strcmp(argv[1], "estimate") != 0) {
        std::cout << "Usage: " << argv[0] << " [calibrate|estimate|board]" << std::endl;
        return -1;
    }

    Estimation estimation(fs);
}

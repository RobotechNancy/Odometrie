//
// Created by mrspaar on 4/25/23.
//

#include "estimation.h"
#include "calibration.h"
#include "robotech/xbee.h"


void print_help() {
    std::cout << "Utilisation :\n"
                 "      ./aruco board <markersX> <markersY> <markerLenght> <markerSeparation>\n"
                 "      ./aruco calib <detectorParamsPath> <markersX> <markersY> <markerLenght> <markerSeparation>\n"
                 "      ./aruco <detectorParamsPath> <cameraParamsPath> <markerLength> <XBeePort>" << std::endl;
}


int main(int argc, char** argv) {
    if (argc == 2 && strcmp(argv[1], "--help") == 0)
        return print_help(), 0;

    if (argc == 6 && strcmp(argv[1], "board") == 0) {
        boardToPng("../board.png", atoi(argv[2]), atoi(argv[3]), atof(argv[4]), atof(argv[5]));
        return 0;
    }

    if (argc == 7 && strcmp(argv[1], "calib") == 0) {
        Camera camera(argv[2]);
        return calibrate(camera, atoi(argv[3]), atoi(argv[4]), atof(argv[5]), atof(argv[6]));
    }

    if (argc != 5)
        return print_help(), 1;

    XBee xbee(argv[4], XB_ADR_ROBOT_01);
    Estimation estimation(argv[1], argv[2], atof(argv[3]));

    if (xbee.openSerialConnection() == 0) {
        xbee.subscribe(XB_FCT_GET_ARUCO_POS, [&xbee, &estimation](const frame_t& frame) {
            estimation.lock([&xbee, &frame](auto ids, auto rVecs, auto tVecs) {
                std::vector<uint8_t> data;

                for (int i=0; i < ids.size(); i++) {
                    data.push_back(ids[i]);
                    data.push_back(rVecs[i][0]);
                    data.push_back(tVecs[i][0]);
                    data.push_back(tVecs[i][1]);
                };

                xbee.sendFrame(frame.adr_emetteur, XB_FCT_ARUCO_POS, data, ids.size()*4);
            });
        });

        xbee.start_listen();
    }

    estimation.start();
    return 0;
}
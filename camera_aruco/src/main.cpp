//
// Created by mrspaar on 4/25/23.
//

#include "estimation.h"
#include "calibration.h"
#include "robotech/xbee.h"


void print_help() {
    std::cout << "Utilisation :\n"
                 "      ./aruco board <markersX> <markersY> <markerLenght> <markerSeparation>\n"
                 "      ./aruco calib <markersX> <markersY> <markerLenght> <markerSeparation>\n"
                 "      ./aruco <markerLength> <port> <camera_param_path>" << std::endl;
}


int main(int argc, char** argv) {
    if (argc == 2 && strcmp(argv[1], "--help") == 0)
        return print_help(), 0;

    if (argc == 6) {
        Calibration calibration(atoi(argv[2]), atoi(argv[3]), atof(argv[4]), atof(argv[5]));

        if (strcmp(argv[1], "board") == 0)
            calibration.boardToPng("../board.png");
        else if (strcmp(argv[1], "calib") == 0)
            calibration.start();

        return 0;
    }

    float markerLen = 0.020;
    if (argc == 2)
        markerLen = atof(argv[1]);

    const char* port = "/dev/ttyUSB0";
    if (argc == 3)
        port = argv[2];

    const char* camera_param_path = "../camera_param.yml";
    if (argc == 4)
        camera_param_path = argv[3];

    XBee xbee(port, XB_ADR_ROBOT_01);
    Estimation estimation(markerLen, camera_param_path);

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
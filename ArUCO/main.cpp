//
// Created by mrspaar on 4/25/23.
//

#include "estimation.h"
#include "calibration.h"


int main(int argc, char** argv) {
    cv::FileStorage fs("../data/config.yml", cv::FileStorage::READ);

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

    //XBee xbee;
    Estimation estimation(fs);

    while (true) {
        estimation.update();
    }

    /*int status = xbee.openSerialConnection(
            fs["xbee_port"].string().c_str(),
            (int) fs["xbee_address"]
    );

    if (status != XB_SER_E_SUCCESS)
        return status;

    xbee.subscribe(XB_FCT_GET_ARUCO_POS, [&estimation, &xbee](const frame_t& frame) {
        estimation.update();
        estimation.send(xbee, frame.adr_emetteur);
    });*/
}

#include "../include/estimator.h"
#include <robotech/xbee.h>

using namespace cv;
using namespace std;
using namespace aruco;

void createBoard(int markersX, int markersY, int markerLength, int markerSeparation) {
    int margins = markerSeparation;
    int borderBits = 1;

    Size imageSize;
    imageSize.width = markersX * (markerLength + markerSeparation) - markerSeparation + 2 * margins;
    imageSize.height = markersY * (markerLength + markerSeparation) - markerSeparation + 2 * margins;

    Dictionary dictionary = getPredefinedDictionary(DICT_4X4_50);
    GridBoard board(Size(markersX, markersY), float(markerLength), float(markerSeparation), dictionary);

    Mat boardImage;
    board.generateImage(imageSize, boardImage, margins, borderBits);

    imshow("board", boardImage);
    waitKey(0);
    imwrite("../board.png", boardImage);
}

int main() {
    Estimator estimator;
    estimator.start();
    return XB_E_SUCCESS;
}

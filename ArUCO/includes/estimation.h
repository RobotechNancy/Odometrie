//
// Created by mrspaar on 4/29/23.
// Modified by garatim & Manal
//

#ifndef OPENCV_ESTIMATION_H
#define OPENCV_ESTIMATION_H

#include "camera.h"         //Inclusion du fichier d'en-tête pour la classe Camera, utilisée pour capturer des images.
#include "robotech/xbee.h"  //Inclusion du fichier d'en-tête pour la classe XBee, utilisée pour l'envoi des données via XBee


/*!
 * @brief Classe pour l'estimation de la position des marqueurs
 * @details Estimation(markerLen)
 */
class Estimation {
public:
    //Constructeur de la classe Estimation
    explicit Estimation(const cv::FileStorage& configFile, int refMid, cv::Vec3d refMarkerPos);

    //Fonctions d'estimation :
    void update();                          //Met à jour les estimations de position des marqueurs
    void send(XBee& xbee, uint8_t dest);    //Envoie les données estimées via XBee
    int ecrituremesures(std::vector<int> ids, std::vector<cv::Vec3d> tVecs);                  //Ecriture des mesures dans un fichier
    
private:
    Camera camera;      //Objet de la classe Camera utilisé pour la capture d'images

    float markerLen;    //Longueur d'un marqueur ArUCO
    int refMarkerId;    //ID du marqueur de référence
    int refMarkerIndex; //Indice du marqueur de référence dans les vecteurs de données

    cv::Vec3d refMarkerPos, origint, originr;   //Positions du marqueur de référence et définition d'origines de translation et rotation

    cv::Mat image, cameraMatrix, distCoeffs;    //Matrices pour stocker l'image capturée, la matrice de la caméra et les coefficients de distorsion
    std::mutex mtx;                             //Mutex pour la synchronisation des accès concurrents aux données partagées

    std::vector<int> ids;                           //Vecteur pour stocker les IDs des marqueurs détectés
    std::vector<std::vector<cv::Point2f>> corners;  //Vecteur pour stocker les coins des marqueurs détectés
    std::vector<cv::Vec3d> rVecs, tVecs;            //Vecteurs pour stocker les vecteurs de rotation et de translation des marqueurs
    std::vector<cv::Vec3d> rAngles, transMat;       //Vecteurs pour stocker les angles de rotation et les coordonnées transformées des marqueurs

};


#endif //OPENCV_ESTIMATION_H


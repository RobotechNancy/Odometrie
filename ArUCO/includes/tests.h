//
// Created by garatim on 09/05/24.
//


#ifndef TESTS_H
#define TEST_H

#include "camera.h"         //Inclusion du fichier d'en-tête pour la classe Camera, utilisée pour capturer des images.

#define Dx 94
#define Dy 94



/*!
 * @brief Classe pour les test 
 * @details Tests(configFile)
 */
class Tests {
public:
    //Constructeur de la classe Tests
    explicit Tests(const cv::FileStorage& configFile, int refMid, cv::Vec3d refMpos);

    //Fonctions de tests :
    // Vérifie la cohérence des coins des marqueurs détectés
    int verifCoherenceArUCO(const std::vector<std::vector<cv::Point2f>>& corners, int i);  
    // Teste la cohérence des marqueurs détectés
    int testCoherence();
    // Vérifie la position de la caméra
    bool cameraPosition();
    // Teste la détection des marqueurs (affiche image avec détections et axes repères)
    int testDetection();
    // Teste la position des marqueurs (affiche coins)
    int testPosition();     
    
private:
    Camera camera;      //Objet de la classe Camera utilisé pour la capture d'images

    float markerLen;        //Longueur d'un marqueur ArUCO
    int refMarkerId;        //ID du marqueur de référence
    int refMarkerIndex;     //Indice du marqueur de référence dans les vecteurs de données
    cv::Vec3d refMarkerPos; //Positions du marqueur de référence

    cv::Mat image, cameraMatrix, distCoeffs;    //Matrices pour stocker l'image capturée, la matrice de la caméra et les coefficients de distorsion
    std::mutex mtx;                             //Mutex pour la synchronisation des accès concurrents aux données partagées

    std::vector<int> ids;                       //Vecteur pour stocker les IDs des marqueurs détectés
    std::vector<std::vector<cv::Point2f>> corners;  //Vecteur pour stocker les coins des marqueurs détectés

};

#endif //TEST_H
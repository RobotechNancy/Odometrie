//
// Created by garatim on 10/05/24.
//


#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "calculsfct.h"


//Variables vecteurs/matrices orientations et translation/position
std::vector<cv::Vec3d> rAngles, transMat;


//Fonction de converion du vecteur de rotation pour obtenir la rotation des codes [Garatim] :
std::vector<cv::Vec3d> RotationTransform(std::vector<cv::Vec3d> rVecs) {
    //convertir les vecteurs rotationnels (rVecs) en matrices de rotation de Rodrigues (rotation matrices)
    std::vector<cv::Mat> rotationMatrices;
    for (int i = 0; i < rVecs.size(); i++) {
        cv::Mat rotationMatrix;
        cv::Rodrigues(rVecs[i], rotationMatrix);
        rotationMatrices.push_back(rotationMatrix);
    }
    //extraire les angles de rotation en degrés à partir des matrices de rotation
    std::vector<cv::Vec3d> rotationAngles;
    for (int i = 0; i < rotationMatrices.size(); i++) {
        cv::Mat rotationMatrix = rotationMatrices[i];
        double theta_x = atan2(rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2)) * 180 / CV_PI;
        double theta_y = atan2(-rotationMatrix.at<double>(2, 0), sqrt(pow(rotationMatrix.at<double>(2, 1), 2) + pow(rotationMatrix.at<double>(2, 2), 2))) * 180 / CV_PI;
        double theta_z = atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0)) * 180 / CV_PI;
        rotationAngles.push_back(cv::Vec3d(theta_x, theta_y, theta_z));
    }
    return rotationAngles;
}







//Fonctions de transformation des données des codes ArUCO pour obtenir leurs coordonnées relative au code de référence [Garatim] :


std::vector<cv::Vec3d> PositionTransform_1(std::vector<int> ids, std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, int refMarkerIndex) {
    transMat.resize(ids.size());
    // création de la matrice de transformation (selon code de référence)
    cv::Mat rotationMatrixD, transformMatrix;
    transformMatrix.create(4, 4, CV_64F);                               //définit la taille de la matrice de transformation
    cv::Rodrigues(rVecs[refMarkerIndex], rotationMatrixD);              //obtient la matrice de rotation issu de rVecs du référent
    transformMatrix.at<double>(0,0) = rotationMatrixD.at<double>(0,0);  //affectation des valeurs dans la mattrice de transformation
    transformMatrix.at<double>(0,1) = rotationMatrixD.at<double>(0,1);  //(cela a été fait séparément pour chaque valeur afin de pouvoir
    transformMatrix.at<double>(0,2) = rotationMatrixD.at<double>(0,2);  //  tester et vérifier, cela pourra être simplifié quand le
    transformMatrix.at<double>(0,3) = tVecs[refMarkerIndex][0];         //  fonctionnement sera fiable)
    transformMatrix.at<double>(1,0) = rotationMatrixD.at<double>(1,0);
    transformMatrix.at<double>(1,1) = rotationMatrixD.at<double>(1,1);
    transformMatrix.at<double>(1,2) = rotationMatrixD.at<double>(1,2);
    transformMatrix.at<double>(1,3) = tVecs[refMarkerIndex][1];
    transformMatrix.at<double>(2,0) = rotationMatrixD.at<double>(0,0);
    transformMatrix.at<double>(2,1) = rotationMatrixD.at<double>(1,1);
    transformMatrix.at<double>(2,2) = rotationMatrixD.at<double>(2,2);
    transformMatrix.at<double>(2,3) = tVecs[refMarkerIndex][2];
    transformMatrix.at<double>(3,0) = 0;
    transformMatrix.at<double>(3,1) = 0;
    transformMatrix.at<double>(3,2) = 0;
    transformMatrix.at<double>(3,3) = 1;

    for (int j=0; j < ids.size(); j++){
        cv::Mat result;
        
        // reformatage de la matrice de translation
        cv::Mat tvec;
        tvec.create(4, 1, CV_64F);
        tvec.at<double>(0,0) = tVecs[j][0];
        tvec.at<double>(1,0) = tVecs[j][1];
        tvec.at<double>(2,0) = tVecs[j][2];
        tvec.at<double>(3,0) = 1;
        // calcul du résultat par la matrice de transformation
        result=transformMatrix.inv()*tvec;
        
        // récupération des coordonnées des codes issues du résultat
        transMat[j][0] = result.at<double>(0);
        transMat[j][1] = result.at<double>(1);
        transMat[j][2] = result.at<double>(2);
    }
    return transMat;
}




//Les fonctions suivantes sont d'autres méthodes qui ont été proposés pour la transfomation des données vecteurs
//(cela permet de pouvoir tester différentes méthodes sans casser la fonction principale)


std::vector<cv::Vec3d> PositionTransform_2(std::vector<int> ids, std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, int refMarkerId) {
    transMat.resize(ids.size());
    cv::Mat camera_to_base_tf;
    for (int j=0; j < ids.size(); j++){
        
        cv::Mat rMat, result;
        cv::Rodrigues(rVecs[j], rMat);
        cv::Mat camera_to_marker_tf = cv::Mat::eye(4, 4, CV_64F);
        rMat.copyTo(camera_to_marker_tf(cv::Rect(0, 0, 3, 3)));
        cv::Mat tvec = cv::Mat(3, 1, CV_64F);
        tvec.at<double>(0, 0) = tVecs[j][0];
        tvec.at<double>(1, 0) = tVecs[j][1];
        tvec.at<double>(2, 0) = tVecs[j][2];
        tvec.copyTo(camera_to_marker_tf(cv::Rect(3, 0, 1, 3)));
        if (ids[j] == refMarkerId) {
            camera_to_base_tf = camera_to_marker_tf.inv();
        }
        cv::Mat marker_in_base_frame = camera_to_base_tf * camera_to_marker_tf;
        cv::Mat translation_vector = marker_in_base_frame(cv::Rect(3, 0, 1, 3));
        result = translation_vector;
        
        // récupération des coordonnées des codes issues du résultat
        transMat[j][0] = result.at<double>(0);
        transMat[j][1] = result.at<double>(1);
        transMat[j][2] = result.at<double>(2);
    }
    return transMat;
}





std::vector<cv::Vec3d> PositionTransform_3(std::vector<int> ids, std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, cv::Mat cameraMatrix, int refMarkerIndex) {
    transMat.resize(ids.size());
    for (int j=0; j < ids.size(); j++){
        cv::Mat rotationMatrixD, result, transformMatrix;
        
        transformMatrix.create(4, 4, CV_64F);
        cv::Rodrigues(rVecs[refMarkerIndex], rotationMatrixD);
        //valeurs incconues estimés
        double U = 0.7;
        double V = 1.05;
        //méthode d'équation s[u v 1]
        cv::Mat tvec = (cv::Mat_<double>(3,1) << tVecs[j][0], tVecs[j][1], tVecs[j][2]);
        cv::Mat uvPoint = (cv::Mat_<double>(3,1) << U, V, 1);
        cv::Mat leftMat = rotationMatrixD.inv() * cameraMatrix.inv() * uvPoint;
        cv::Mat rightMat = rotationMatrixD.inv() * tvec;
        double s = (0 + leftMat.at<double>(2,0))/rightMat.at<double>(2,0);
        result = rotationMatrixD.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);
        
        //récupération des coordonnées des codes issues du résultat
        transMat[j][0] = result.at<double>(0);
        transMat[j][1] = result.at<double>(1);
        transMat[j][2] = result.at<double>(2);
    }
    return transMat;
}

 



std::vector<cv::Vec3d> PositionTransform_4(std::vector<int> ids, std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, int refMarkerIndex, int refMarkerId) {
    std::vector<cv::Vec3d> transMat;
    transMat.resize(ids.size());
    cv::Mat rotationMatrixRef, transformMatrixRef;
    cv::Rodrigues(rVecs[refMarkerIndex], rotationMatrixRef);
    //Initialisation de la matrice de transformation pour le marqueur de référence
    transformMatrixRef = cv::Mat::eye(4, 4, CV_64F);
    rotationMatrixRef.copyTo(transformMatrixRef(cv::Rect(0, 0, 3, 3)));
    //Copier les valeurs du vecteur de translation dans la matrice de transformation du marqueur de référence
    transformMatrixRef.at<double>(0, 3) = tVecs[refMarkerIndex][0];
    transformMatrixRef.at<double>(1, 3) = tVecs[refMarkerIndex][1];
    transformMatrixRef.at<double>(2, 3) = tVecs[refMarkerIndex][2];

    for (int j = 0; j < ids.size(); j++) {
        if (ids[j] != refMarkerId) {
            cv::Mat tvecMarker;
            tvecMarker.create(4, 1, CV_64F);
            tvecMarker.at<double>(0, 0) = tVecs[j][0];
            tvecMarker.at<double>(1, 0) = tVecs[j][1];
            tvecMarker.at<double>(2, 0) = tVecs[j][2];
            tvecMarker.at<double>(3, 0) = 1;

            cv::Mat transformedMarker = transformMatrixRef.inv() * tvecMarker;
            transMat[j][0] = transformedMarker.at<double>(0, 0);  // x value
            transMat[j][1] = transformedMarker.at<double>(1, 0);  // y value
            transMat[j][2] = transformedMarker.at<double>(2, 0);  // z value
        } else {
            //SItuer le marqueur de référence à (0, 0, 0) dans le nouveau système de coordonnées.
            transMat[j][0] = 0.0;
            transMat[j][1] = 0.0;
            transMat[j][2] = 0.0;
        }
    }
    return transMat;
}





std::vector<cv::Vec3d> PositionTransform_5(std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, int refMarkerIndex) {
    std::vector<cv::Vec3d> relativePositions;
    // Obtenir la matrice de rotation du marqueur de référence
    cv::Mat rMat_ref, transVec_ref;
    cv::Rodrigues(rVecs[refMarkerIndex], rMat_ref);
    // Obtenir le vecteur de translation du marqueur de référence
    cv::Mat tVec_ref(tVecs[refMarkerIndex]);
    // Calculer la transformation inverse pour le marqueur de référence
    cv::Mat rotMat_ref_inv = rMat_ref.t(); // Transposition de l'inverse de la matrice de rotation
    cv::Mat transVec_ref_inv = -rotMat_ref_inv * tVec_ref;
    for (size_t i = 0; i < rVecs.size(); ++i) {
        // Passer le marqueur de référence lui-même
        if (i == refMarkerIndex)
            continue;

        cv::Mat rMat, transVec;
        // Obtenir la matrice de rotation pour le marqueur actuel
        cv::Rodrigues(rVecs[i], rMat);
        // Obtenir le vecteur de translation pour le marqueur actuel
        transVec = cv::Mat(tVecs[i]);
        // Transformer le vecteur de translation dans le système de coordonnées du marqueur de référence
        cv::Mat transformedTransVec = rotMat_ref_inv * transVec + transVec_ref_inv;
        // Enregistrer la position relative du marqueur actuel
        relativePositions.push_back(cv::Vec3d(transformedTransVec));
        
        cv::Mat rMat_curr, transVec_curr;
        cv::Rodrigues(rVecs[i], rMat_curr);
        cv::Mat tVec_curr(tVecs[i]);

        // Transformer le vecteur de translation du marqueur actuel dans le système de coordonnées du marqueur de référence
        cv::Mat tVec_curr_ref = rotMat_ref_inv * tVec_curr + transVec_ref_inv;

        // Enregistre la position relative du marqueur actuel
        relativePositions.push_back(cv::Vec3d(tVec_curr_ref));
    }
    return relativePositions;
}





std::vector<cv::Vec3d> PositionTransform_6(std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, int refMarkerIndex) {
    // Calculer la matrice de transformation
    cv::Mat R_ref;
    cv::Rodrigues(rVecs[refMarkerIndex], R_ref);
    cv::Mat T_ref = cv::Mat(tVecs[refMarkerIndex]);
    cv::Mat transformMatrix = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat rotationMatrix = cv::Mat::eye(3, 3, CV_64F);
    // Remplir la partie de rotation
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotationMatrix.at<double>(i, j) = R_ref.at<double>(i, j);
            transformMatrix.at<double>(i, j) = R_ref.at<double>(i, j);
        }
    }
    // Remplir l'la partie de translation
    for (int i = 0; i < 3; ++i) {
        transformMatrix.at<double>(i, 3) = T_ref.at<double>(i);
    }
    // Remplir la dernière ligne
    transformMatrix.at<double>(3, 3) = 1.0;
    cv::Mat rotationTransposed;
    cv::transpose(rotationMatrix, rotationTransposed);
    // Combiner la rotation et la translation en une seule matrice de transformation
    cv::Mat rotationTransposedWithT = rotationTransposed * T_ref;
    rotationTransposedWithT.copyTo(transformMatrix(cv::Rect(0, 3, 3, 1)));

    // Transformer chaque position de marqueur en système de coordonnées de base
    for (size_t i = 0; i < rVecs.size(); ++i) {
        cv::Mat rVec = cv::Mat(rVecs[i]);
        cv::Mat tVec = cv::Mat(tVecs[i]);
        cv::Mat markerPos = cv::Mat::zeros(4, 1, CV_64F);

        //rVec.copyTo(markerPos(cv::Rect(0, 0, 3, 1)));
        //tVec.copyTo(markerPos(cv::Rect(0, 3, 1, 1)));
        for (int j = 0; j < 3; ++j) {
            markerPos.at<double>(j, 0) = rVec.at<double>(j, 0);
        }
        for (int j = 0; j < 3; ++j) {
            markerPos.at<double>(j, 0) = tVec.at<double>(j, 0);
        }

        markerPos.at<double>(3, 0) = 1.0;
        cv::Mat markerPosBase = transformMatrix * markerPos;
        transMat.push_back(cv::Vec3d(markerPosBase.at<double>(0, 0),
                                      markerPosBase.at<double>(1, 0),
                                      markerPosBase.at<double>(2, 0)));
    }
    return transMat;
}

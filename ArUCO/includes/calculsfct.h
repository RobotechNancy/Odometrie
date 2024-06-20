//
// Created by garatim on 10/05/24.
//


#ifndef CALCULSFCT_H
#define CALCULSFCT_H



//Fonction de converion du vecteur de rotation pour obtenir la rotation des codes :
std::vector<cv::Vec3d> RotationTransform(std::vector<cv::Vec3d> rVecs);

//Fonctions de transformation des données des codes ArUCO pour obtenir leurs coordonnées relative au code de référence :
std::vector<cv::Vec3d> PositionTransform_1(std::vector<int> ids, std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, int refMarkerIndex);
std::vector<cv::Vec3d> PositionTransform_2(std::vector<int> ids, std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, int refMarkerId);
std::vector<cv::Vec3d> PositionTransform_3(std::vector<int> ids, std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, cv::Mat cameraMatrix, int refMarkerIndex);
std::vector<cv::Vec3d> PositionTransform_4(std::vector<int> ids, std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, int refMarkerIndex, int refMarkerId);
std::vector<cv::Vec3d> PositionTransform_5(std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, int refMarkerIndex);
std::vector<cv::Vec3d> PositionTransform_6(std::vector<cv::Vec3d> rVecs, std::vector<cv::Vec3d> tVecs, int refMarkerIndex);



#endif //CALCULSFCT_H
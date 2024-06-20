//
// Created by mrspaar on 4/25/23.
// Modified by garatim & Manal
//


//Inclusions faisant liens avec les programmes
#include "estimation.h"
#include "calibration.h"
#include "tests.h"




int main(int argc, char** argv) {
    //Initialisation du chemin d'accès au fichier de configuration utilisé dans les programmes
    cv::FileStorage fs("../data/config.yml", cv::FileStorage::READ);



    //Si l'argument d'exécution est "calibrate", lancement du programme de calibration de la caméra
    if (argc > 1 && strcmp(argv[1], "calibrate") == 0) {
        return calibrate(fs);
    }

    //Si l'argument d'exécution est "board", lancement du programme de création du pateau de calibration (cadre imprimé)
    if (argc > 1 && strcmp(argv[1], "board") == 0) {
        return boardToPng(fs), 0;
    }



    //Initialisation de la variable du choix d'équipe (valeur à récupérer : 0=jaune, 1=bleu)
    bool team = 0;


    //Initialisation des variables pour les informations du marqueur de reférence : son id et ses coordonnées sur le plateau de jeu
    int refMarkerId;
    cv::Vec3d refMarkerPos;

    //Récupération des informations du marqueur de reférence en fonction de l'équipe
    if (team==0) {
        refMarkerId = (int) fs["ref_marker_id_yellow"];
	    fs["ref_marker_pos_yellow"] >> refMarkerPos;
    }
    else if (team==1) {
        refMarkerId = (int) fs["ref_marker_id_blue"];
        fs["ref_marker_pos_blue"] >> refMarkerPos;
    }



    //Création d'objets "Estimation" et "Tests" avec attributs : le fichier de configuration et les informations sur le marqueur de référence
    //  Tests tests(fs, refMarkerId, refMarkerPos);
    //  Estimation estimation(fs, refMarkerId, refMarkerPos);
    //posés dans l'appel des fonctions/programmes car problèmes de duplication d'ouverture de la caméra causant des erreurs





    //Si l'argument d'exécution est "testCam", lancement du programme de test du positionnement de la caméra/balise (si bon poteau placé au bon endroit)
    if (argc > 1 && strcmp(argv[1], "testCam") == 0) {
        Tests tests(fs, refMarkerId, refMarkerPos);
        if(tests.cameraPosition()){
		    std::cout<<"Correct Position"<<std::endl;
            return 0;
   	    }
	    else{
            std::cout<<"Incorrect Position"<<std::endl;
            return -1;
        }
    }


    //Si l'argument d'exécution est "testDetection", lancement du programme de test de la détection de codes (Affichage)
    if (argc > 1 && strcmp(argv[1], "testDetection") == 0) {
        Tests tests(fs, refMarkerId, refMarkerPos);
        return tests.testDetection();
    }

    //Si l'argument d'exécution est "testPosition", lancement du programme de test de la position de codes (Coords coins 1)
    if (argc > 1 && strcmp(argv[1], "testPosition") == 0) {
        Tests tests(fs, refMarkerId, refMarkerPos);
        return tests.testPosition();
    }

    //Si l'argument d'exécution est "testCoherence", lancement du programme de test de la position de codes
    if (argc > 1 && strcmp(argv[1], "testCoherence") == 0) {
        Tests tests(fs, refMarkerId, refMarkerPos);
        return tests.testCoherence();
    }





    //Si l'argument d'exécution est "estimateTest", lancement du programme de test de l'estimation (sans envoi de données)
    if (argc > 1 && strcmp(argv[1], "estimateTest") == 0) {
        Estimation estimation(fs, refMarkerId, refMarkerPos);
        estimation.update();
        return 0;
    }



    //Si l'argument d'exécution n'est pas une valeur voulue, message d'erreur avec affichage des arguments valables
    if (argc < 2 || strcmp(argv[1], "estimate") != 0) {
        std::cout << "Usage: " << argv[0] << " [calibrate|estimate|board|testCam|estimateTest|testDetection|testPosition|testCoherence]" << std::endl;
        return -1;
    }




    //Si l'argument d'exécution est "estimate", lancement du programme d'estimation de la position des codes détectés et envoie des données au robot
    
    Estimation estimation(fs, refMarkerId, refMarkerPos); //objet estimation

    // crée un objet XBee avec l'adresse spécifiée
    XBee xbee(XB_ADDR_CAMERA_01);
    // ouvre une connexion XBee en utilisant le port spécifié dans les paramètres du fichier de configuration
    int status = xbee.open(fs["xbee_port"].string().c_str());

    // vérifie si l'ouverture de la connexion XBee a réussi
    // si la Xbee a rencontré de l'erreur, retourne l'état
    if (status != XB_E_SUCCESS)
        return status;

    // lie une fonction à un événement spécifique de la XBee :
    //Chaque fois que la XBee reçoit un message avec le type "XB_FCT_GET_ARUCO_POS" 
    //la fonction estimation.update() est appelée pour mettre à jour l'estimation à l'instant, 
    //et la fonction estimation.send() est ensuite appelée pour envoyer une réponse au Robot.
    xbee.bind(XB_FCT_GET_ARUCO_POS, [&estimation](XBee &xbee, const xbee_frame_t &frame) {
        estimation.update();
        estimation.send(xbee, frame.emitterAddress);
    });

    // démarre l'écoute des messages entrants sur la XBee
    xbee.startListening();
    // met le thread en pause pendant 10 secondes, ce qui permet à la XBee de rester à l'écoute pendant cette période
    std::this_thread::sleep_for(std::chrono::seconds(10));

    return 0;




    /* XBee xbee;
    Estimation estimation(fs);

    while (true) {
        estimation.update();
    }

    int status = xbee.openSerialConnection(
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

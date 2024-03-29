/*!
 * 	\file can.h
 *  \brief     classe de gestion d'un bus can
 *  \details   Cette classe permet d'envoyer et de recevoir des messages via un bus can
 *  \author    Theo RUSINOWITCH <theo.rusinowitch1@etu.univ-lorraine.fr>
 *  \version   4.1a
 *  \date      2021-2022
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdbool.h>
#include "defineCan.h"
#include "stm32l4xx_hal.h"

typedef unsigned int uint;

typedef struct {
	unsigned char *data;
	char len;
} GraphState_t;


typedef struct {
	uint addr;	/* addresse du destinataire du message */
	uint emetteur;	/* adresse de l'éméteur */
	uint codeFct;	/* code fonction du msg */
	uint idMessage;
	bool isRep;	/* vrai si c'est une reponse a une requete, faux sinon */
	uint RepId;	/* nb de rep atendu si requete, num de la rep si reponse */
	//Tramme_Can_extID_t ExtID;
	uint dataLen;	/* frame payload length in byte (0 .. 8) */
	unsigned char data[8];
} CanResponse_t;

CanResponse_t traitement_trame(CAN_RxHeaderTypeDef frame, uint8_t data[]);
void CAN_Config(CAN_HandleTypeDef hcan, CAN_EMIT_ADDR adresse);
int send(CAN_ADDR addr, CAN_CODE_FCT codeFct , uint8_t data[], uint data_len, bool isRep, uint repLenght, uint idMessage);
int start_listen();

#endif /* INC_CAN_H_ */

/*!
 * 	\file can.h
 *  \brief     classe de gestion d'un bus can
 *  \details   Cette classe permet d'envoyer et de recevoir des messages via un bus can
 *  \author    Theo RUSINOWITCH <theo.rusinowitch1@etu.univ-lorraine.fr>
 *  \version   4.1a
 *  \date      2021-2022
 */

#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "can_vars.h"

void configure_CAN(CAN_HandleTypeDef hcan, CAN_EMIT_ADDR adresse);
int format_frame(can_mess_t *msg, CAN_RxHeaderTypeDef frame, const uint8_t data[]);
int send(CAN_ADDR addr, CAN_FCT_CODE fct_code , uint8_t data[], uint8_t data_len, bool is_rep, uint8_t rep_len, uint8_t msg_id);

#endif /* CAN_H */

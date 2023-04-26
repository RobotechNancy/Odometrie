/*
 * TOF_Units.c
 *
 *  Created on: May 8, 2022
 *      Author: Samuel
 */

#include "../Platform/TOF.h"
#include "TOF_Units.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

VL53L0X_Dev_t TOF_Devices[] = {
	{
		.Id = TOF_UNIT_0,
		.I2cHandle = &hi2c1,
		.XSHUT_Port = TOF0_XSHUT_GPIO_Port,
		.XSHUT_Pin = TOF0_XSHUT_Pin
	},
	{
		.Id = TOF_UNIT_1,
		.I2cHandle = &hi2c1,
		.XSHUT_Port = TOF1_XSHUT_GPIO_Port,
		.XSHUT_Pin = TOF1_XSHUT_Pin
	},
	{
		.Id = TOF_UNIT_2,
		.I2cHandle = &hi2c1,
		.XSHUT_Port = TOF2_XSHUT_GPIO_Port,
		.XSHUT_Pin = TOF2_XSHUT_Pin
	},
	{
		.Id = TOF_UNIT_3,
		.I2cHandle = &hi2c1,
		.XSHUT_Port = TOF3_XSHUT_GPIO_Port,
		.XSHUT_Pin = TOF3_XSHUT_Pin
	},
	{
		.Id = TOF_UNIT_4,
		.I2cHandle = &hi2c1,
		.XSHUT_Port = TOF4_XSHUT_GPIO_Port,
		.XSHUT_Pin = TOF4_XSHUT_Pin
	},
	{
		.Id = TOF_UNIT_5,
		.I2cHandle = &hi2c1,
		.XSHUT_Port = TOF5_XSHUT_GPIO_Port,
		.XSHUT_Pin = TOF5_XSHUT_Pin
	},
	{
		.Id = TOF_UNIT_6,
		.I2cHandle = &hi2c1,
		.XSHUT_Port = TOF6_XSHUT_GPIO_Port,
		.XSHUT_Pin = TOF6_XSHUT_Pin
	},
	{
		.Id = TOF_UNIT_7,
		.I2cHandle = &hi2c1,
		.XSHUT_Port = TOF7_XSHUT_GPIO_Port,
		.XSHUT_Pin = TOF7_XSHUT_Pin
	},
//	{
//		.Id = TOF_UNIT_8,
//		.I2cHandle = &hi2c1,
//		.XSHUT_Port = TOF8_XSHUT_GPIO_Port,
//		.XSHUT_Pin = TOF8_XSHUT_Pin
//	}
};



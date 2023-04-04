/*
 * Accelero.h
 *
 *  Created on: May 21, 2022
 *      Author: Samuel
 */

#ifndef ACCELERO_H_
#define ACCELERO_H_

// public includes

// public defines

// public macros

// public types

typedef enum {
	ACCEL_NO_DATA = 0,
	ACCEL_DATA_AVAILABLE
} Accelero_NewData_t;


typedef enum {
	ACCEL_CALIBRATION_AT_STARTUP = 0,
	ACCEL_SKIP_CALIBRATION
} Accelero_Calibration_t;

typedef struct {
	uint16_t mean_etimation_samples_number;
	Accelero_Calibration_t skip_calibration;
} Accelero_Config_t;

typedef struct {
	Accelero_Config_t * Config;
} Accelero_Desciptor_t;

// public functions

void Accelero_Init(Accelero_Desciptor_t * _Descriptor);

void Accelero_Process(void);

void Accelero_Calibrate(void);
bool Accelero_isReady(void);

float Accelero_GetAccX(void);
void Accelero_SetAccX(float newValue);
//
float Accelero_GetAccY(void);
void Accelero_SetAccY(float newValue);
//
//float Accelero_GetAccZ(void);
//void Accelero_SetAccZ(float newValue);

float Accelero_GetCalibrationAdvancement(void);

Accelero_NewData_t Accelero_NewData(void);

#endif /* ACCELERO_H_ */

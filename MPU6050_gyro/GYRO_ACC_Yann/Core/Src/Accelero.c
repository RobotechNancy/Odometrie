/*
 * Accelero.c
 *
 *  Created on: Jan 17, 2023
 *      Author: Yann
 */

// private includes
#include "MPU6050.h"
#include "Accelero.h"

// private defines

// private macros

// private types

//typedef enum {
//	ACCEL_NO_DATA = 0,
//	ACCEL_DATA_AVAILABLE
//} Accelero_NewData_t;

typedef enum {
	ACCEL_STATE_NOT_INIT = 0,
	ACCEL_STATE_INIT,
	ACCEL_STATE_CALIBRATION,
	ACCEL_STATE_APPLY_CALIBRATION,
	ACCEL_STATE_READY
} Accelero_State_t;

typedef struct {
	double accumulator;
	uint16_t counter;
	double offset_x;
	double offset_y;
	double Accelerorate;
	double datarate;
} Accelero_Data_t;

// private variables

Accelero_Desciptor_t * Descriptor_Acc;

Accelero_Data_t Accelero_Data;

Accelero_State_t State_Acc = ACCEL_STATE_NOT_INIT;

// private functions


void Accelero_GetData_X(void)
{
	Accelero_Data.Accelerorate = MPU6050_GetData_CF_Accelero(0) - Accelero_Data.offset_x;
}
void Accelero_GetData_Y(void)
{
	Accelero_Data.Accelerorate = MPU6050_GetData_CF_Accelero(1) - Accelero_Data.offset_y;
}

Accelero_NewData_t Accelero_NewData(void)
{
	return (Accelero_NewData_t)MPU6050_NewData();
}

void Accelero_ResetAccumulator(void)
{
	Accelero_Data.accumulator = 0.0;
	Accelero_Data.counter = 0;
}

void Accelero_Accumulate_X(void)
{
	if(Accelero_NewData()==ACCEL_DATA_AVAILABLE)
	{
		Accelero_GetData_X();
		Accelero_Data.accumulator += Accelero_Data.Accelerorate;
		Accelero_Data.counter++;
	}
}

void Accelero_Accumulate_Y(void)
{
	if(Accelero_NewData()==ACCEL_DATA_AVAILABLE)
	{
		Accelero_GetData_Y();
		Accelero_Data.accumulator += Accelero_Data.Accelerorate;
		Accelero_Data.counter++;
	}
}

void Accelero_SetOffset_X(void)
{
	Accelero_Data.offset_x = Accelero_Data.accumulator / Accelero_Data.counter;
}

void Accelero_SetOffset_Y(void)
{
	Accelero_Data.offset_y = Accelero_Data.accumulator / Accelero_Data.counter;
}

void Accelero_Estimate_X(void)
{
	if(Accelero_NewData()==ACCEL_DATA_AVAILABLE)
	{
		Accelero_GetData_X();
	}
}

void Accelero_Estimate_Y(void)
{
	if(Accelero_NewData()==ACCEL_DATA_AVAILABLE)
	{
		Accelero_GetData_Y();
	}
}

void Accelero_FSM(void)
{
	// actions
	switch(State_Acc)
	{
	case ACCEL_STATE_NOT_INIT :
		// NOP
		break;

	case ACCEL_STATE_INIT :
		Accelero_ResetAccumulator();
		break;

	case ACCEL_STATE_CALIBRATION :
		Accelero_Accumulate_X();
		Accelero_Accumulate_Y();
		break;

	case ACCEL_STATE_APPLY_CALIBRATION :
		Accelero_SetOffset_X();
		Accelero_SetOffset_Y();
		break;

	case ACCEL_STATE_READY:
		Accelero_Estimate_X();
		Accelero_Estimate_Y();
		break;

	default :
		break;
	}

	// transitions
	switch(State_Acc)
	{
	case ACCEL_STATE_NOT_INIT :
		// NOP
		break;

	case ACCEL_STATE_INIT :
		if( Descriptor_Acc->Config->skip_calibration )
			State_Acc = ACCEL_STATE_READY;
		else
			State_Acc = ACCEL_STATE_CALIBRATION;
		break;

	case ACCEL_STATE_CALIBRATION :
		if( Accelero_Data.counter >= Descriptor_Acc->Config->mean_etimation_samples_number - 1 )
			State_Acc = ACCEL_STATE_APPLY_CALIBRATION;
		break;

	case ACCEL_STATE_APPLY_CALIBRATION :
		State_Acc = ACCEL_STATE_READY;
		break;

	case ACCEL_STATE_READY:
		// NOP
		break;

	default :
		break;
	}
}

// public functions implementations

void Accelero_Init(Accelero_Desciptor_t * _Descriptor)
{
	Descriptor_Acc = _Descriptor;
	Accelero_Data.datarate = MPU6050_GetSampleRate();
	Accelero_ResetAccumulator();
	Accelero_SetAccX(0);
	Accelero_SetAccX(0);
	State_Acc = ACCEL_STATE_INIT;
}

void Accelero_Process(void)
{
	Accelero_FSM();
}

void Accelero_Calibrate(void){
	Accelero_ResetAccumulator();
	State_Acc = ACCEL_STATE_CALIBRATION;
}

bool Accelero_isReady(void)
{
	return State_Acc == ACCEL_STATE_READY;
}

float Accelero_GetAccX(void)
{
	return MPU6050_GetData_CF_Accelero(0)- Accelero_Data.offset_x;
}

void Accelero_SetAccX(float newValue)
{
	Accelero_Data.Accelerorate = newValue;
}

float Accelero_GetAccY(void)
{
	return MPU6050_GetData_CF_Accelero(1)- Accelero_Data.offset_y;
}

void Accelero_SetAccY(float newValue)
{
	Accelero_Data.Accelerorate = newValue;
}

//float Accelero_GetAccZ(void)
//{
//	return Accelero_Data;
//}
//
//void Accelero_SetAccZ(float newValue)
//{
//	Accelero_Data = newValue;
//}

float Accelero_GetCalibrationAdvancement(void)
{
	return (float)(Accelero_Data.counter) * 100 / (float)(Descriptor_Acc->Config->mean_etimation_samples_number);
}

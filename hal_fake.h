#ifndef _HAL_FAKE_H_
#define _HAL_FAKE_H_

#include "hal.h"

/* Controls the simulated pack. Not part of the real HAL --
   test code only. BMS.c must never include this header. */

void FAKE_Reset(void);
void FAKE_SetPackHealthy(void);
void FAKE_SetCellVoltage(int cell, float volts);
void FAKE_SetCellTemp(int cell, float degC);

void FAKE_AdvanceMS(uint32_t ms);

/* Stages a frame and calls RxCan(), standing in for the
   hardware interrupt. */
void FAKE_SendIsense(int32_t milliamps);
void FAKE_SendHeartbeat(void);
void FAKE_SendFaultsClear(void);

bool     FAKE_GetSDC(void);
void     FAKE_GetLastCan(uint16_t* id, uint8_t data[CAN_LEN]);
void     FAKE_PrintStatus(const char* label);

#endif
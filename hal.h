
#ifndef _HAL_H_
#define _HAL_H_
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#define CAN_LEN 8
#define N_CELLS 130
void HAL_ReadVoltages(float data[N_CELLS]);
void HAL_ReadTemperatures(float data[N_CELLS]);
void HAL_SetSDC(bool closed);
void HAL_SetLED(uint8_t r, uint8_t g, uint8_t b);
void HAL_SendCanMsg(uint16_t id, const uint8_t data[CAN_LEN]);
void HAL_RecvCanMsg(uint16_t* id, uint8_t data[CAN_LEN]);
uint32_t HAL_GetMS(void);
#endif
 
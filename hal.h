#ifndef _HAL_H_
#define _HAL_H_


#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>


#define CAN_LEN 8 // A CAN message has 8 data bytes
#define N_CELLS 130


// Reads the voltage data for all the cells in volts
void HAL_ReadVoltages(float data[N_CELLS]);


// Reads the temperature data for all the cells in deg C
void HAL_ReadTemperatures(float data[N_CELLS]);


// Sets the state of the Shutdown Circuit
void HAL_SetSDC(bool closed);


// Sets a status LED to an RGB color (0-255 each)
void HAL_SetLED(uint8_t r, uint8_t g, uint8_t b);


// Send a CAN message onto the bus
void HAL_SendCanMsg(uint16_t id, const uint8_t data[CAN_LEN]);


// Read a CAN message from the bus.
// The ID is written into *id and the data into the buffer.
void HAL_RecvCanMsg(uint16_t* id, uint8_t data[CAN_LEN]);


// Gets the time in milliseconds since the controller was powered on.
uint32_t HAL_GetMS(void);


#endif

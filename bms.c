#include "hal.h"


void Init()
{
   // This function runs once on startup


}


void Iter()
{
   // This function runs periodically at ~20Hz


}


void RxCan()
{
   // Called every time a CAN frame is received on the bus, using an interrupt.
   // Keep in mind, this can be called at any point in the execution of your program.
   // You may not use any HAL_* functions here except HAL_RecvCanMsg,
   // which is how you can pull the message from the bus.
   // An example for pulling a CAN frame is shown below.


   uint8_t data[CAN_LEN];
   uint16_t id;
   HAL_RecvCanMsg(&id, data);
}

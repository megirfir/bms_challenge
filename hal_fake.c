#include "hal.h"
#include "hal_fake.h"
#include <stdio.h>
#include <string.h>

/* RxCan lives in BMS.c. Declared here so the fake interrupt
   can call it. */
void RxCan(void);

/* ---- the simulated world ---- */
static float    fake_v[N_CELLS];
static float    fake_t[N_CELLS];
static bool     fake_sdc;
static uint32_t fake_ms;
static uint8_t  led_r, led_g, led_b;

static uint16_t pending_id;
static uint8_t  pending_data[CAN_LEN];

static uint16_t last_tx_id;
static uint8_t  last_tx_data[CAN_LEN];

/* ================= HAL implementations ================= */

void HAL_ReadVoltages(float data[N_CELLS])
{
    for (int i = 0; i < N_CELLS; i++) data[i] = fake_v[i];
}

void HAL_ReadTemperatures(float data[N_CELLS])
{
    for (int i = 0; i < N_CELLS; i++) data[i] = fake_t[i];
}

void HAL_SetSDC(bool closed)
{
    fake_sdc = closed;
}

void HAL_SetLED(uint8_t r, uint8_t g, uint8_t b)
{
    led_r = r; led_g = g; led_b = b;
}

void HAL_SendCanMsg(uint16_t id, const uint8_t data[CAN_LEN])
{
    last_tx_id = id;
    memcpy(last_tx_data, data, CAN_LEN);
}

void HAL_RecvCanMsg(uint16_t* id, uint8_t data[CAN_LEN])
{
    *id = pending_id;
    memcpy(data, pending_data, CAN_LEN);
}

uint32_t HAL_GetMS(void)
{
    return fake_ms;
}

/* ================= test controls ================= */

void FAKE_Reset(void)
{
    memset(fake_v, 0, sizeof(fake_v));
    memset(fake_t, 0, sizeof(fake_t));
    fake_sdc = false;
    fake_ms  = 0;
    memset(last_tx_data, 0, CAN_LEN);
    last_tx_id = 0;
}

void FAKE_SetPackHealthy(void)
{
    for (int i = 0; i < N_CELLS; i++) {
        fake_v[i] = 3.700f + (float)(i % 7) * 0.005f;
        fake_t[i] = 25.0f  + (float)(i % 11) * 0.5f;
    }
}

void FAKE_SetCellVoltage(int cell, float volts)
{
    if (cell >= 0 && cell < N_CELLS) fake_v[cell] = volts;
}

void FAKE_SetCellTemp(int cell, float degC)
{
    if (cell >= 0 && cell < N_CELLS) fake_t[cell] = degC;
}

void FAKE_AdvanceMS(uint32_t ms)
{
    fake_ms += ms;
}

static void inject(uint16_t id, const uint8_t* payload)
{
    pending_id = id;
    memset(pending_data, 0, CAN_LEN);
    if (payload) memcpy(pending_data, payload, CAN_LEN);
    RxCan();
}

void FAKE_SendIsense(int32_t milliamps)
{
    uint8_t d[CAN_LEN];
    memset(d, 0, CAN_LEN);
    d[2] = (uint8_t)((milliamps >> 24) & 0xFF);
    d[3] = (uint8_t)((milliamps >> 16) & 0xFF);
    d[4] = (uint8_t)((milliamps >> 8)  & 0xFF);
    d[5] = (uint8_t)( milliamps        & 0xFF);
    inject(0x511, d);
}

void FAKE_SendHeartbeat(void)   { inject(0x1CD, NULL); }
void FAKE_SendFaultsClear(void) { inject(0x1CF, NULL); }

bool FAKE_GetSDC(void) { return fake_sdc; }

void FAKE_GetLastCan(uint16_t* id, uint8_t data[CAN_LEN])
{
    *id = last_tx_id;
    memcpy(data, last_tx_data, CAN_LEN);
}

void FAKE_PrintStatus(const char* label)
{
    const uint8_t* d = last_tx_data;
    printf("%-32s t=%5ums  SDC=%-6s  active=%02X latched=%02X  "
           "min=%4dmV max=%4dmV  flags=%02X  LED=%3d,%3d,%3d\n",
           label, fake_ms, fake_sdc ? "CLOSED" : "OPEN",
           d[0], d[1],
           (d[2] << 8) | d[3], (d[4] << 8) | d[5],
           d[6], led_r, led_g, led_b);
}
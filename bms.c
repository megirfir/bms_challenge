#include "hal.h"
#include <math.h>

/* ------------------------------------------------------------------
 * Fault bits. One bit per fault so a single byte carries any
 * combination of simultaneous faults.
 * ------------------------------------------------------------------ */
#define F_OV        0x01    /* cell over voltage      */
#define F_UV        0x02    /* cell under voltage     */
#define F_OT        0x04    /* cell over temperature  */
#define F_DELTA     0x08    /* pack imbalance         */
#define F_OC        0x10    /* pack over current      */
#define F_IMPL      0x20    /* implausible cell data  */
#define F_ISENSE    0x40    /* current sensor timeout */

#define N_FAULTS    7

/* ---- thresholds from the spec ---- */
#define V_MAX_V         4.2f
#define V_MIN_V         2.5f
#define T_MAX_C         60.0f
#define DELTA_MAX_V     0.2f
#define I_MAX_MA        200000L

/* ---- plausibility band (assumption, see design doc) ---- */
#define V_PLAUS_LO_V    0.5f
#define V_PLAUS_HI_V    5.5f
#define T_PLAUS_LO_C    (-40.0f)
#define T_PLAUS_HI_C    125.0f

/* ---- timing ---- */
#define DEBOUNCE_N          2       /* consecutive scans, ~100ms at 20Hz */
#define STARTUP_DWELL_MS    500u
#define STARTUP_CLEAN_SCANS 2
#define ISENSE_TIMEOUT_MS   500u    /* 5 missed frames at 10Hz */
#define DIAG_TIMEOUT_MS     3000u
#define I_REST_MA           2000L   /* +/- 2A counts as "at rest" */

/* ---- CAN ---- */
#define ID_BMS_STATUS   0x0B0
#define ID_ISENSE       0x511
#define ID_DIAG_HB      0x1CD
#define ID_FAULTS_CLEAR 0x1CF

/* Debounce requirement per fault, indexed by bit position.
 * Voltage and temperature come from raw measurements and are
 * debounced. Current arrives pre-filtered from a dedicated CAN
 * device, so it is trusted on first reading. The sensor timeout
 * is itself a 500ms filter. */
static const uint8_t fault_required[N_FAULTS] = {
    DEBOUNCE_N,  /* F_OV     */
    DEBOUNCE_N,  /* F_UV     */
    DEBOUNCE_N,  /* F_OT     */
    DEBOUNCE_N,  /* F_DELTA  */
    1,           /* F_OC     */
    DEBOUNCE_N,  /* F_IMPL   */
    1            /* F_ISENSE */
};

/* ------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------ */
static uint8_t  active;
static uint8_t  latched;
static uint8_t  debounce[N_FAULTS];

static bool     startup_done;
static uint8_t  clean_scans;
static uint32_t boot_ms;

static uint32_t last_isense_ms;
static uint32_t last_diag_ms;
static bool     isense_ever;
static bool     diag_ever;

static uint8_t  rolling_counter;

/* Written by RxCan (interrupt context), read by Iter. */
static volatile bool     rx_clear_req;
static volatile bool     rx_isense_new;
static volatile bool     rx_diag_new;
static volatile int32_t  rx_isense_ma;

/* ------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------ */

/* Torn-read guard: rx_isense_ma is 32 bits and RxCan can fire between
 * halves of a read on a narrow core. Read until two reads agree. */
static int32_t snapshot_current(void)
{
    int32_t a, b;
    do {
        a = rx_isense_ma;
        b = rx_isense_ma;
    } while (a != b);
    return a;
}

static bool ms_elapsed(uint32_t now, uint32_t since, uint32_t limit)
{
    return (uint32_t)(now - since) >= limit;
}

static uint16_t volts_to_mv(float v)
{
    if (v <= 0.0f) return 0;
    if (v >= 65.0f) return 65000;
    return (uint16_t)(v * 1000.0f + 0.5f);
}

/* ------------------------------------------------------------------
 * Init
 * ------------------------------------------------------------------ */
void Init(void)
{
    int i;

    /* SDC is normally open. Nothing has been measured yet, so the
     * safe state is commanded explicitly rather than assumed. */
    HAL_SetSDC(false);
    HAL_SetLED(0, 0, 255);

    active = 0;
    latched = 0;
    for (i = 0; i < N_FAULTS; i++) debounce[i] = 0;

    startup_done = false;
    clean_scans = 0;
    boot_ms = HAL_GetMS();

    last_isense_ms = 0;
    last_diag_ms = 0;
    isense_ever = false;
    diag_ever = false;
    rolling_counter = 0;

    rx_clear_req = false;
    rx_isense_new = false;
    rx_diag_new = false;
    rx_isense_ma = 0;
}

/* ------------------------------------------------------------------
 * Iter  -  runs at ~20Hz
 * ------------------------------------------------------------------ */
void Iter(void)
{
    float v[N_CELLS];
    float t[N_CELLS];
    uint32_t now;
    uint8_t raw;
    int32_t current_ma;
    float vmin, vmax;
    int valid_cells;
    int i;
    bool isense_fresh, diag_present, sdc_close;
    uint8_t msg[CAN_LEN];
    uint16_t mv_min, mv_max;

    now = HAL_GetMS();

    /* --- consume interrupt flags. RxCan may not call HAL_GetMS, so
     *     arrival times are stamped here instead. --- */
    if (rx_isense_new) {
        rx_isense_new = false;
        last_isense_ms = now;
        isense_ever = true;
    }
    if (rx_diag_new) {
        rx_diag_new = false;
        last_diag_ms = now;
        diag_ever = true;
    }

    current_ma = snapshot_current();
    isense_fresh = isense_ever && !ms_elapsed(now, last_isense_ms, ISENSE_TIMEOUT_MS);
    diag_present = diag_ever && !ms_elapsed(now, last_diag_ms, DIAG_TIMEOUT_MS);

    /* --- read the pack --- */
    HAL_ReadVoltages(v);
    HAL_ReadTemperatures(t);

    /* --- evaluate raw conditions for this scan --- */
    raw = 0;
    vmin = 0.0f;
    vmax = 0.0f;
    valid_cells = 0;

    for (i = 0; i < N_CELLS; i++) {
        bool v_ok = !isnan(v[i]) && v[i] >= V_PLAUS_LO_V && v[i] <= V_PLAUS_HI_V;
        bool t_ok = !isnan(t[i]) && t[i] >= T_PLAUS_LO_C && t[i] <= T_PLAUS_HI_C;

        if (!v_ok || !t_ok) {
            /* A NaN reading passes every threshold comparison, so a
             * dead channel would otherwise look like a healthy cell. */
            raw |= F_IMPL;
            continue;
        }

        if (v[i] > V_MAX_V) raw |= F_OV;
        if (v[i] < V_MIN_V) raw |= F_UV;
        if (t[i] > T_MAX_C) raw |= F_OT;

        if (valid_cells == 0 || v[i] < vmin) vmin = v[i];
        if (valid_cells == 0 || v[i] > vmax) vmax = v[i];
        valid_cells++;
    }

    if (valid_cells >= 2 && (vmax - vmin) > DELTA_MAX_V) raw |= F_DELTA;
    if (valid_cells == 0) raw |= F_IMPL;

    if (isense_fresh && current_ma > I_MAX_MA) raw |= F_OC;
    if (isense_ever && !isense_fresh) raw |= F_ISENSE;

    /* --- debounce --- */
    active = 0;
    for (i = 0; i < N_FAULTS; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        if (raw & bit) {
            if (debounce[i] < 255) debounce[i]++;
            if (debounce[i] >= fault_required[i]) active |= bit;
        } else {
            debounce[i] = 0;
        }
    }

    /* --- latch --- */
    latched |= active;

    /* --- clear request ---
     * Stricter than the spec: the spec only requires that no fault is
     * active. A clear is a pit operation, so a diagnostic tool must be
     * connected and the pack must be at rest with a fresh reading. */
    if (rx_clear_req) {
        rx_clear_req = false;
        if (active == 0 &&
            diag_present &&
            isense_fresh &&
            current_ma <= I_REST_MA && current_ma >= -I_REST_MA) {
            latched = 0;
            /* Re-verify before energising again. */
            startup_done = false;
            clean_scans = 0;
        }
    }

    /* --- startup gate ---
     * Not a fault. A fault means something is wrong; these checks are
     * expected to pass. The pack does not energise until dwell time has
     * elapsed, two consecutive scans are clean, and real current data
     * has been received. */
    if (!startup_done) {
        if (active == 0 && latched == 0) {
            if (clean_scans < 255) clean_scans++;
        } else {
            clean_scans = 0;
        }

        if (clean_scans >= STARTUP_CLEAN_SCANS &&
            isense_fresh &&
            ms_elapsed(now, boot_ms, STARTUP_DWELL_MS)) {
            startup_done = true;
        }
    }

    /* --- shutdown circuit ---
     * Driven off the latch, not the live state, so a fault that has
     * since cleared still holds the car down. Acted on before the CAN
     * transmit: opening the relay must not wait on bus arbitration. */
    sdc_close = startup_done && (latched == 0);
    HAL_SetSDC(sdc_close);

    /* --- status LED --- */
    if (active != 0)            HAL_SetLED(255, 0, 0);      /* fault now      */
    else if (latched != 0)      HAL_SetLED(255, 80, 0);     /* latched only   */
    else if (!startup_done)     HAL_SetLED(0, 0, 255);      /* verifying      */
    else                        HAL_SetLED(0, 255, 0);      /* running        */

    /* --- BMS_STATUS frame --- */
    mv_min = (valid_cells > 0) ? volts_to_mv(vmin) : 0;
    mv_max = (valid_cells > 0) ? volts_to_mv(vmax) : 0;

    rolling_counter = (uint8_t)((rolling_counter + 1) & 0x0F);

    msg[0] = active;
    msg[1] = latched;
    msg[2] = (uint8_t)(mv_min >> 8);
    msg[3] = (uint8_t)(mv_min & 0xFF);
    msg[4] = (uint8_t)(mv_max >> 8);
    msg[5] = (uint8_t)(mv_max & 0xFF);
    msg[6] = rolling_counter;
    if (sdc_close)    msg[6] |= 0x10;
    if (startup_done) msg[6] |= 0x20;
    if (diag_present) msg[6] |= 0x40;
    if (isense_fresh) msg[6] |= 0x80;
    msg[7] = 0x00;   /* reserved */

    HAL_SendCanMsg(ID_BMS_STATUS, msg);
}

/* ------------------------------------------------------------------
 * RxCan  -  interrupt context
 *
 * Records facts only. Every decision is made in Iter, where the whole
 * picture exists. No HAL call other than HAL_RecvCanMsg is permitted
 * here, so incoming frames cannot be timestamped in this function.
 * ------------------------------------------------------------------ */
void RxCan(void)
{
    uint8_t data[CAN_LEN];
    uint16_t id;

    HAL_RecvCanMsg(&id, data);

    if (id == ID_ISENSE) {
        rx_isense_ma = ((int32_t)data[2] << 24)
                     | ((int32_t)data[3] << 16)
                     | ((int32_t)data[4] << 8)
                     |  (int32_t)data[5];
        rx_isense_new = true;
    } else if (id == ID_FAULTS_CLEAR) {
        rx_clear_req = true;
    } else if (id == ID_DIAG_HB) {
        rx_diag_new = true;
    }
}
#include <stdio.h>
#include <string.h>
#include "hal.h"
#include "hal_fake.h"

extern void Init(void);
extern void Iter(void);

static int g_pass = 0, g_fail = 0;

static void check(const char *label, bool got, bool want) {
    printf("    [%s] got=%-5s want=%-5s -> %s\n", label,
           got ? "true" : "false", want ? "true" : "false",
           (got == want) ? "PASS" : "FAIL");
    if (got == want) g_pass++; else g_fail++;
}

static uint8_t last_active(void) {
    uint16_t id; uint8_t d[CAN_LEN];
    FAKE_GetLastCan(&id, d);
    return d[0];
}
static uint8_t last_latched(void) {
    uint16_t id; uint8_t d[CAN_LEN];
    FAKE_GetLastCan(&id, d);
    return d[1];
}

/* Runs a normal startup (dwell + 2 clean scans + live current data) so
 * scenarios can begin from a closed SDC in the running state. */
static void boot_to_running(void) {
    FAKE_Reset();
    Init();
    FAKE_SetPackHealthy();
    for (int i = 0; i < 15; i++) {           /* > 500ms dwell */
        FAKE_SendIsense(0);                  /* real sensor posts ~10Hz; keep fresh */
        FAKE_AdvanceMS(50);
        Iter();
    }
}

int main(void) {
    printf("AME27 BMS -- test harness (real hal.h/hal_fake.h interface)\n\n");

    printf("1. Safe default at power-on (no data yet)\n");
    FAKE_Reset();
    Init();
    Iter();
    FAKE_PrintStatus("  cold boot, one scan");
    check("SDC open before startup proven", FAKE_GetSDC(), false);

    printf("2. Normal startup completes, SDC closes\n");
    boot_to_running();
    FAKE_PrintStatus("  after boot sequence");
    check("SDC closed after clean startup", FAKE_GetSDC(), true);

    printf("3. Cell over-voltage, 2-scan debounce\n");
    boot_to_running();
    FAKE_SetCellVoltage(0, 4.3f);
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    check("not latched after 1 scan", (last_latched() & 0x01) != 0, false);
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    check("latched after 2 scans", (last_latched() & 0x01) != 0, true);
    check("SDC opens on latch", FAKE_GetSDC(), false);

    printf("4. Single-scan glitch does not fault\n");
    boot_to_running();
    FAKE_SetCellVoltage(0, 4.3f);
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    FAKE_SetCellVoltage(0, 3.7f); /* glitch clears before 2nd scan */
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    check("no fault from single glitch", (last_latched() & 0x01) != 0, false);

    printf("5. Cell under-voltage\n");
    boot_to_running();
    FAKE_SetCellVoltage(5, 2.3f);
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    check("under-voltage latched", (last_latched() & 0x02) != 0, true);

    printf("6. Cell over-temperature\n");
    boot_to_running();
    FAKE_SetCellTemp(10, 65.0f);
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    check("over-temp latched", (last_latched() & 0x04) != 0, true);

    printf("7. Cell delta exceeded\n");
    boot_to_running();
    FAKE_SetCellVoltage(0, 3.9f);
    FAKE_SetCellVoltage(1, 3.6f);
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    check("delta latched", (last_latched() & 0x08) != 0, true);

    printf("8. Pack over-current, no debounce\n");
    boot_to_running();
    FAKE_SendIsense(250000); /* 250A in mA, threshold is 200A */
    FAKE_AdvanceMS(50); Iter();
    check("over-current latched after 1 scan", (last_latched() & 0x10) != 0, true);

    printf("9. NaN cell voltage caught by plausibility check\n");
    boot_to_running();
    FAKE_SetCellVoltage(20, 0.0f / 0.0f /* NaN */);
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    check("implausible latched", (last_latched() & 0x20) != 0, true);
    check("NaN cell did not also raise over/under-voltage", (last_active() & 0x03) != 0, false);

    printf("10. ISENSE_TIMEOUT after 500ms silence\n");
    boot_to_running();
    for (int i = 0; i < 15 && !((last_latched() & 0x40) != 0); i++) {
        FAKE_AdvanceMS(50);
        Iter(); /* no FAKE_SendIsense -- sensor has gone silent */
    }
    check("isense timeout latched by ~500ms", (last_latched() & 0x40) != 0, true);

    printf("11. Fault clears only when gated (tool present + pack at rest)\n");
    boot_to_running();
    FAKE_SetCellVoltage(0, 4.3f);
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    check("faulted", (last_latched() & 0x01) != 0, true);
    FAKE_SetCellVoltage(0, 3.7f); /* condition resolved */
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    FAKE_SendFaultsClear(); /* clear requested with no diag tool present */
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    check("clear rejected, no heartbeat", (last_latched() & 0x01) != 0, true);
    FAKE_SendHeartbeat();
    FAKE_SendFaultsClear();
    FAKE_SendIsense(0); FAKE_AdvanceMS(50); Iter();
    check("clear accepted with tool present + pack at rest", (last_latched() & 0x01) != 0, false);
    /* Clearing resets the startup gate, so it takes a couple more clean
     * scans before the SDC recloses -- same as a real re-verification. */
    for (int i = 0; i < 5; i++) {
        FAKE_SendHeartbeat();
        FAKE_SendIsense(0);
        FAKE_AdvanceMS(50);
        Iter();
    }
    check("SDC recloses after re-verification", FAKE_GetSDC(), true);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
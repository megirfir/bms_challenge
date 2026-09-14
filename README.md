# AME27 Embedded Systems Design Challenge — BMS

A fault-handling Battery Management System (BMS) designed for an EV racecar.

This project implements the safety logic for a **130-cell series battery pack**, monitoring cell voltage, cell temperature, and pack current. When a safety fault is detected and latched, the BMS opens the shutdown circuit (SDC) and continues reporting the fault over CAN for the driver and diagnostics.

## Overview

The BMS monitors:

- Cell voltage
- Cell temperature
- Pack current
- CAN communication health
- Sensor plausibility

It implements the five required faults plus two additional safety faults:

| Fault | Condition | Debounce |
|---|---|---:|
| `CELL_OVER_VOLTAGE` | Any cell > 4.2 V | 2 scans |
| `CELL_UNDER_VOLTAGE` | Any cell < 2.5 V | 2 scans |
| `CELL_OVER_TEMPERATURE` | Any cell > 60 °C | 2 scans |
| `CELL_DELTA_EXCEEDED` | Max − min cell voltage > 0.2 V | 2 scans |
| `PACK_OVER_CURRENT` | Discharge current > 200 A | Immediate |
| `CELL_IMPLAUSIBLE` | NaN or physically impossible sensor data | 2 scans |
| `ISENSE_TIMEOUT` | No `0x511` frame for 500 ms | Immediate |

The main control loop runs every **50 ms (20 Hz)**.

---

## State Machine

The BMS operates in four states:

### 1. Startup

The car has powered on but has not yet passed its energizing checks.

**SDC: OPEN**

Startup requires:

- 500 ms settling time
- Two consecutive clean sensor scans
- At least one valid current-sensor reading

Startup is treated as a state rather than a fault, so a laptop is not required simply to start the car.

### 2. Monitoring

All startup checks have passed and the battery is considered safe to energize.

**SDC: CLOSED**

The BMS continuously monitors the battery and looks for faults.

### 3. Faulted

One or more faults have been latched.

**SDC: OPEN**

CAN communication continues so that the driver dash and diagnostic tools can still identify why the shutdown circuit opened.

Latched faults remain until a valid clearing procedure is completed.

### 4. Clear Pending

A `FAULTS_CLEAR` (`0x1CF`) request has been received, but the battery has not yet passed the requirements to re-energize.

**SDC: unchanged**

---

## Main Control Loop

Every 50 ms, `Iter()` performs the following sequence:

1. Record CAN data arrival times and read the current sensor
2. Fetch all cell voltages and temperatures
3. Check fault conditions
4. Apply debounce filters
5. Latch newly detected faults
6. Process fault-clear requests
7. Perform startup safety checks
8. Command the SDC
9. Update the diagnostic LED
10. Transmit the final `BMS_STATUS` CAN frame

The ordering is intentional.

### Safety-critical ordering

Faults are latched **before** a clear request is evaluated.

Therefore, if a fault occurs during the same 50 ms scan as a clear request, the fault wins and the clear request is ignored.

The SDC is also commanded **before** CAN transmission, so a delayed CAN bus cannot prevent the shutdown action.

The SDC depends on the **latched fault list**, not the temporary active fault list. This prevents a transiently clean reading from automatically re-energizing the car after a fault.

---

## Fault Handling

### Voltage and Temperature Faults

Voltage and temperature faults require **two consecutive failed scans**.

At 20 Hz, this gives a worst-case debounce time of **100 ms**.

This prevents a single noisy reading or electrical glitch from unnecessarily shutting down the car.

### Pack Overcurrent

Pack overcurrent is handled differently.

The current sensor provides pre-filtered data, so an overcurrent condition is triggered on the first bad reading.

```text
current > 200 A
        ↓
PACK_OVER_CURRENT
        ↓
Latch fault
        ↓
Open SDC
```

A scan-based debounce would not provide meaningful filtering here because the current sensor reports at 10 Hz while `Iter()` runs at 20 Hz. Each reading would normally appear in two consecutive scans anyway.

> **Note:** The current implementation only considers dangerously high **discharge** current. Regenerative braking / charging current is not treated as an overcurrent condition.

---

## Sensor Plausibility

One important failure mode is a dead temperature or voltage sensor returning `NaN`.

A naive comparison such as:

```c
if (voltage > 4.2)
```

does not catch `NaN`, because comparisons against `NaN` evaluate to false.

That could make a failed sensor appear healthy.

The BMS therefore performs an explicit plausibility check.

Sensor values outside the expected hardware range are also considered implausible:

- Voltage: **0.5–5.5 V**
- Temperature: **−40–125 °C**

Invalid data results in a latched `CELL_IMPLAUSIBLE` fault and prevents the vehicle from energizing.

These ranges are intended as hardware-failure detection rather than a complete real-world sensor validation strategy.

---

## Fault Clearing

A fault cannot simply be cleared by sending a CAN message.

To accept `FAULTS_CLEAR` (`0x1CF`), the following conditions must be satisfied:

1. There are no active faults
2. A diagnostic laptop/tool is physically connected
3. The tool has remained connected for 3 seconds
4. The car is physically at rest
5. Pack current is near zero
6. The current reading is fresh

This prevents an arbitrary CAN node, accidental message, or communication error from clearing a dangerous battery fault while the car is moving.

After clearing, the BMS **re-verifies the battery before closing the SDC**.

---

## CAN Interface

### BMS Status — `BMS_STATUS`

The BMS broadcasts its status at **20 Hz**.

| Byte | Field | Description |
|---:|---|---|
| 0 | Active faults | Fault bitmask |
| 1 | Latched faults | Latched fault bitmask |
| 2–3 | Minimum cell voltage | Unsigned 16-bit integer, mV |
| 4–5 | Maximum cell voltage | Unsigned 16-bit integer, mV |
| 6 | Counter + status | Rolling counter and system flags |
| 7 | Reserved | Always `0` |

### Status Byte

Byte 6 contains:

```text
Bits 0–3 : Rolling counter (0–15)
Bit 4    : SDC status
Bit 5    : Startup status
Bit 6    : Diagnostic tool connected
Bit 7    : Sensor data fresh
```

### Rolling Counter

The rolling counter increments from 0–15 and then wraps around.

This provides a simple way for other systems to detect a crashed or frozen BMS.

If the counter stops changing, another controller can determine that the BMS is no longer operating correctly and take appropriate safety action.

---

## Received CAN Frames

### `0x511 — ISENSE_DATA`

Contains:

- Pack current
- Direction of power flow
- Diagnostic-tool presence

Current is reported in milliamps.

The BMS also monitors this frame for communication timeout.

If at least one frame has previously been received and then no frame arrives for **500 ms**, `ISENSE_TIMEOUT` is triggered.

Because CAN frames cannot be timestamped directly inside the interrupt handler, the timeout's worst-case response is approximately **550 ms**.

### `0x1CF — FAULTS_CLEAR`

Requests that latched faults be cleared.

The request is only accepted if all safety requirements are satisfied.

---

## Interrupt / Concurrency Design

`RxCan()` runs as an interrupt.

The interrupt handler deliberately does **not** perform safety decisions.

Instead, it:

1. Receives the CAN frame
2. Stores the data
3. Sets the appropriate flag
4. Returns

All actual safety decisions happen inside `Iter()`.

This keeps the safety logic centralized and prevents the SDC from being controlled unpredictably from an interrupt.

### Consistent Current Reading

Because an interrupt can occur while `Iter()` is running, the current value is copied into a local variable once at the beginning of the scan.

The rest of the scan uses that same value.

### `volatile`

Variables shared between `RxCan()` and `Iter()` are marked `volatile` so the compiler cannot incorrectly assume that they never change.

### Torn Reads

For shared 32-bit data, the BMS reads the value twice and compares the results.

```text
Read value A
Read value B

A == B ?
 ├── Yes → use value
 └── No  → retry
```

This prevents a partially updated 32-bit value from being interpreted as a valid measurement.

---

## Testing

The implementation was tested against 11 scenarios with **17 assertions**, all of which passed.

Test cases include:

- Safe default at power-on
- Normal startup
- Cell over-voltage
- Single-scan voltage glitch
- Cell under-voltage
- Cell over-temperature
- Cell voltage delta exceeded
- Pack overcurrent
- NaN cell voltage
- ISENSE timeout
- Fault clearing

One startup issue was discovered during testing:

Initially, `ISENSE_TIMEOUT` could trigger before the current sensor had transmitted its first frame, causing every power-on to fault.

The fix was to only evaluate the timeout **after at least one `ISENSE_DATA` frame has been received**.

This allows startup to wait for the sensor instead of interpreting the initial absence of data as a communication failure.

---

## Design Decisions

### Why transmit only minimum and maximum cell voltage?

The BMS reports the minimum and maximum cell voltages rather than all 130 individual cell voltages.

During a race, knowing the exact failed cell does not provide much actionable information because the battery cannot realistically be repaired mid-race.

Minimum and maximum voltage provide useful information about overall pack health while keeping the BMS status message compact.

Detailed diagnostics can be performed later in the pits.

### Why use a rolling counter?

A frozen CAN message can be dangerous.

If the BMS crashes after transmitting a healthy status, another system could continue seeing a message that says everything is fine.

The rolling counter makes a stale message detectable.

### Why debounce voltage and temperature?

These signals can be affected by electrical noise, especially in an EV environment with high-power motors.

Two consecutive bad readings provide a 100 ms filter against isolated glitches while still responding quickly enough for the intended design.

### Why require a physical diagnostic tool to clear faults?

A CAN message alone is not strong evidence that a human has intentionally inspected the vehicle.

Requiring a diagnostic tool, a three-second connection period, zero/near-zero pack current, and no active faults creates a deliberate physical reset procedure.

---

## Bus Usage

The BMS transmits one CAN frame every 50 ms.

The estimated bandwidth usage is approximately:

**2,600 bits/s**, or roughly **0.5% of a 500 kbps CAN bus** for the BMS status traffic.

For comparison, transmitting all 130 cell voltages as unsigned 16-bit values would require approximately 33 CAN frames per update, using roughly **17% of the CAN bandwidth at 20 Hz**.

This was another reason to keep the primary status frame compact.

---

## Assumptions & Limitations

This implementation is a design challenge submission rather than a production-ready automotive BMS.

Important limitations include:

- The 100 ms BMS response time has not been verified against the applicable rulebook.
- `HAL_ReadVoltages()` and `HAL_ReadTemperatures()` are assumed to complete within the 50 ms scan period.
- The plausibility ranges are intended to detect major hardware failures and would need more validation for a real vehicle.
- Mid-`Iter()` interrupt behavior has been designed for but was not fully verified by the test harness.
- Testing was performed using simulated values rather than physical hardware.
- `PACK_OVER_CURRENT` only monitors discharge current, not regenerative charging current.
- Transmitting every individual cell voltage would significantly increase CAN bus utilization.

---

## Project Structure

The implementation is contained in the linked GitHub repository:

**[github.com/megirfir/bms_challenge](https://github.com/megirfir/bms_challenge)**

The code implements:

- BMS state management
- Fault detection
- Fault debounce
- Fault latching
- Fault clearing
- Startup checks
- SDC control
- CAN RX/TX handling
- Sensor plausibility checks
- Diagnostic status reporting
- Test scenarios

---

## Tools Used

| Component | Used for |
|---|---|
| C | BMS implementation |
| Claude Opus 5 | Design/implementation assistance |
| Claude Sonnet 5 | Test-scenario code assistance |

The design decisions and reasoning behind the implementation are my own and I am able to walk through the code and explain the reasoning behind each major decision.

---

## Summary

The central safety principle of this BMS is:

> **Detect faults quickly, latch them permanently, and require deliberate human intervention before re-energizing the vehicle.**

The system prioritizes deterministic safety behavior over convenience:

- The SDC fails open.
- Faults are latched.
- Fault clearing requires physical presence.
- Startup requires independent safety checks.
- CAN communication cannot delay the shutdown action.
- Sensor failures are treated as faults rather than healthy readings.
- A rolling counter allows other systems to detect a frozen BMS.

This project was developed as a technical design challenge for an EV racecar BMS.
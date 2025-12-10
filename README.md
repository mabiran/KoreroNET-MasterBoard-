# KōreroNET  — Firmware Overview & UART Protocol

This repository contains the STM32U0 firmware that runs the low-power KōreroNET field node. It controls the AudioMoth recorder, talks to a Raspberry Pi over UART, schedules duty-cycled recording (audible/ultrasonic modes), monitors power using an INA219 shunt monitor and an ADC battery divider, and keeps a rolling 25-hour power history for the Pi to fetch.

------------------------------------------------------------
## Features at a glance
------------------------------------------------------------
- **UART control link (115200-8-N-1)** to Raspberry Pi with line-oriented commands and human-readable responses.
- **Recording scheduler** with per-hour modes: Standby (0), Audible (S), Ultrasonic (U), and Process/Pause (P), including automatic toggling of the AudioMoth AM_REC pin.
- **Pi wake signaling** via RPI_WAKE pulse during processing mode, with a 5-hour pause window.
- **Power monitoring** with INA219 (voltage/current) and ADC fallback (voltage-only).
- **Coulomb counting**, SoC_i & SoC_v estimation, FULL (end-of-charge) detection.
- **Rolling 25-hour power history** (Wh, mAh, SoC_i, SoC_v) printable via UART.
- **Snapshot reporting** of live system metrics for remote Pi logging.

------------------------------------------------------------
## Hardware mapping
------------------------------------------------------------
- UART1: PA9/PA10 (TX/RX), IRQ enabled, baud 115200.
- Battery ADC: PA0 → ADC1_IN4; 12-bit conversion.
- INA219 via I2C: continuous bus/shunt measurement.
- GPIOs:
  - AM_REC: Record trigger line to AudioMoth (LOW = record).
  - AM_CONFIG: Audio front-end selector (HIGH = audible, LOW = ultrasonic).
  - RPI_WAKE: Wake pulse to Raspberry Pi during processing.
  - RPISwitch and other auxiliary GPIOs as defined in `pins_config.h`.

------------------------------------------------------------
## Build
------------------------------------------------------------
This is an STM32Cube HAL-based firmware project targeting STM32U083RC.  
Generated files: `adc.c`, `usart.c`, `gpio.c`, `stm32u0xx_hal_msp.c`, `stm32u0xx_it.c`.

Interrupts: SysTick, USART1, EXTI. HAL-driven event handling ensures low latency for UART commands and sensor sampling.

------------------------------------------------------------
## UART protocol (Pi ⇄ Nucleo)
------------------------------------------------------------
**Physical:** 115200 baud, 8-N-1, ASCII text, lines end with CRLF.

### Commands from Raspberry Pi
- `nucleostartrecording` → Start immediate recording (according to current mode).
- `nucleogivemetimetable` → Print current 24-hour timetable string (S/U/0/P).
- `nucleoidonotneedultrasound` → Force audible mode.
- `nucleoineedultrasound` → Force ultrasonic mode.
- `nucleoineedfiles` → Pause scheduler 5h, pulse RPI_WAKE, notify Pi.
- `nucleopowerstats` → Print power snapshot (voltage, current, SoC_i/v).
- `nucleopowerhistory` → Print 4 CSV rows (PH_WH, PH_mAh, PH_SoCi, PH_SoCv).
- `wake me up in <h>` → Schedule wake pulse after <h> hours.
- `nucleosettime DD/MM/YYYY HH:MM:SS` → Set RTC time (if RTC active).

### Events sent from STM32
- `EVENT: schedule paused for processing`
- `Vsrc[INA]` snapshot line
- 4 CSV rows for 25-hour power history

------------------------------------------------------------
## Scheduler behaviour
------------------------------------------------------------
Each hour is one of:
- S → Audible recording (AM_CONFIG HIGH; toggles AM_REC on/off)
- U → Ultrasonic recording (AM_CONFIG LOW; toggles AM_REC on/off)
- 0 → Standby (no recording)
- P → Processing (wakes Pi, pauses 5h)

Transitions occur at hour boundaries. The scheduler maintains:
`g_sched_recording`, `g_sched_next_toggle_ms`, and `g_sched_hour`.

------------------------------------------------------------
## Power management
------------------------------------------------------------
- INA219 (continuous mode) provides voltage/current readings.
- ADC fallback (ADC1_IN4) samples battery voltage via divider.
- Coulomb counter integrates current for mAh and Wh history.
- SoC_i = 100 × (1 – used/NOMINAL_CAPACITY).
- SoC_v estimated from voltage map.
- FULL detection when: Vsrc ≥ 14.2 V, |I| ≤ 0.15 A, dwell confirmed.

------------------------------------------------------------
## Power history
------------------------------------------------------------
- 25-hour rolling history arrays: Wh, mAh, SoC_i, SoC_v.
- Printed as CSV with 25 comma-separated values per line.
Example:
```
PH_WH,0.000123,...,0.000987
PH_mAh,1.234567,...,2.345678
PH_SoCi,97.0,...,96.0
PH_SoCv,98.0,...,97.0
```

------------------------------------------------------------
## GPIO configuration
------------------------------------------------------------
GPIOs configured as push-pull outputs. Power pins and wake lines are grouped logically:
- AM_REC → toggled during record/sleep cycles.
- AM_CONFIG → audio mode selector.
- RPI_WAKE → pulse output for Pi.
All mapping resides in `pins_config.h`.

------------------------------------------------------------
## Interrupts
------------------------------------------------------------
- SysTick → Millisecond timing for scheduler.
- USART1 IRQ → Handles UART RX/TX buffer events.
- EXTI → Reserved for user events/wakeups.

------------------------------------------------------------
## Example serial session
------------------------------------------------------------
```
$ picocom -b 115200 /dev/ttyUSB0
nucleopowerstats
Vsrc[INA]=13.812 V | I=-0.221 A | used=532/12000 mAh | SoC_i=96% | SoC_v=98% | FULL_MARKED

nucleopowerhistory
PH_WH,0.0001,...
PH_mAh,1.2,...
PH_SoCi,97,...
PH_SoCv,98,...

nucleoineedfiles
EVENT: schedule paused for processing
```

------------------------------------------------------------
## Developer notes
------------------------------------------------------------
- INA219 preferred for live readings; ADC used if unavailable.
- FULL marking resets coulomb integrator for accurate SoC.
- UART1_Send() is used throughout for serial prints.
- Scheduler and power logic fully decoupled for modular debugging.

------------------------------------------------------------
## File map (selected)
------------------------------------------------------------
- main.c — Scheduler, UART, power logic, EoC handling.
- ina219.c — Minimal INA219 driver (no calibration register).
- battery_adc.c — ADC-based voltage measurement.
- adc.c, gpio.c, usart.c, stm32u0xx_hal_msp.c, stm32u0xx_it.c — HAL peripherals.

------------------------------------------------------------
## Typical Pi workflow
------------------------------------------------------------
1. Send timetable (24-char day string of S/U/0/P).
2. Node follows hourly schedule, toggling AM_REC accordingly.
3. When Pi must process data, send `nucleoineedfiles`.
4. Query power snapshot or history anytime for diagnostics.

------------------------------------------------------------
## License
------------------------------------------------------------
Polyform Noncommercial License 1.0.0

You may use, copy, and modify this firmware only for non-commercial purposes.  
Attribution required. No warranty.

(c) 2025

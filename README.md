# Home-Automation HV3.0 — Firmware FV5.0.0 — Full Documentation

> Repository folder: `Home-Automation-HV3.0-FV5.0.0` — hardware version HV3.0, firmware version FV5.0.0. (HV = hardware version; FV = firmware version)

⚙️ **Overview**

[![License: CC-BY-NC-4.0](https://img.shields.io/badge/License-CC--BY--NC_4.0-red.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

This project is licensed under [CC BY-NC 4.0](./LICENSE) — personal and non-commercial use permitted. Commercial distribution (selling devices that include this firmware) is prohibited without express permission. Read `COMMERCIAL_LICENSE.md` for options to request a commercial license. To request a commercial license, please open a 'Commercial license request' issue using the template in `.github/ISSUE_TEMPLATE/commercial-license-request.md`.

---

## Table of Contents

0. [Includes & Why](#0-includes--why-)
1. [Highlights & Features](#1-highlights--features-)
2. [Build-time Constraints & Safety Checks](#2-build-time-constraints--safety-checks-️)
3. [Hardware Pins & Wiring](#3-hardware-pins--wiring-)
4. [Timing & Key Constants](#4-timing--key-constants-️)
5. [Zero-cross Detection & Triac Gating](#5-zero-cross-detection--triac-gating-flow-)
6. [Timer Operation & ISRs](#6-timer-operation--isrs-️)
7. [Controls, IR & Serial Commands](#7-controls-ir--serial-commands-)
8. [7-Segment Display Mapping](#8-7-segment-display-mapping-)
9. [EEPROM & Persistent States](#9-eeprom--persistent-states-)
10. [Diagnostics, Watchdog & Auto-Recovery](#10-diagnostics-watchdog--auto-recovery-️)
11. [Building & Flashing](#11-building--flashing-️)
12. [Test & Troubleshooting Steps](#12-test--troubleshooting-steps-)
13. [Known Issues & Suggested Fixes](#13-known-issues--suggested-fixes-️)
14. [Future Improvements & Ideas](#14-future-improvements--ideas-)

---

## 0) Includes & Why 📦

- `EEPROM.h`: Read/write persistent settings (fan level, states) via `EEPROM.read()`/`EEPROM.update()`.
- `util/atomic.h`: `ATOMIC_BLOCK()` to safely access volatile variables from ISRs and main code.
- `avr/pgmspace.h`: Store/lookup constant tables (`PROGMEM`) to reduce RAM usage; `pgm_read_*` helpers.
- `avr/interrupt.h`: `ISR()` macros, `sei()`, and interrupt handling helpers.
- `avr/io.h`: Low-level register and port definitions (DDR/PORT/TIMER registers, PD3, PC5, etc.).
- `string.h` / `stdlib.h`: `strncmp()` and `atoi()` used by the serial MINP command parsing.

This explanation helps maintainers know why each include is present and what code relies on it.

---

## What's new in v5.0 ✨
- Version bump from 4.0 to 5.0: internal cleanup and documentation updates.
- Added MINP command (case-insensitive) to query/set minimum fan conduction percent and persisted to EEPROM.
- Simplified serial command set: removed legacy AT+ prefixes and removed testing toggles such as 'k'.
- Always-on, concise diagnostics via Serial at 115200.
- Fan-level mapping improved for more uniform power steps; default min conduction set to 5%.
 - Added IR command debounce (120 ms) to avoid processing repeated IR presses immediately.
 - Pending fan changes are now scheduled (short debounce) to reduce motor/transient spikes on rapid changes.
 - Auto-recovery behavior has been tuned to resume automatically if zero-cross reappears within a short window, and an improved triac fire parity counters are logged for diagnostics.

---

## 1) Highlights & Features ✅
- Zero-cross detection (INT0) for AC sync (CT354A style)
- Triac gating with Timer1 in CTC mode — non-blocking, precise gate pulses
- 9-level fan speed control via phase-angle delay (1–9)
- EEPROM-backed persistent states for power, fan, lights, and socket
- IR remote decoding using a 12-bit SIRC-like protocol with PB2 sampling
- Diagnostic & debug logging over Serial (always enabled in firmware)
- Watchdog-like zero-cross monitoring with auto-recovery
- Direct port manipulation and ATmega8-specific register usage
- `MINP?` - Query the minimum conduction percent for fan level 1 (0..100).
- `MINP=NN` - Set and persist the minimum conduction percent for fan level 1 (NN between 0 and 100).
    - When `MINP` is set it will be stored in EEPROM and the target ticks will be recalculated immediately if the fan and power are currently enabled.
  - New in v5: Commands are case-insensitive; `MINP` replaces legacy `AT+MINP` format.

---

## 2) Build-time Constraints & Safety Checks ⚠️
- Only supports ATmega8 / ATmega8A:
  - `#if !defined(__AVR_ATmega8__) && !defined(__AVR_ATmega8A__)` -> compile-time error
- `F_CPU` must be 16 MHz:
  - `#if (F_CPU != 16000000UL)` -> compile-time error
- Please note: In this firmware version, debug logging is always enabled and cannot be disabled at build or runtime.
  - Optional: if you need to reduce build size, gate `Serial` diagnostics via a compile-time `#define` (e.g., `#define DIAG 1`) and wrap `LOG/LOGS` macros with `#if DIAG`.

---

## 3) Hardware pins & wiring 🔧
- CPU: ATmega8 / ATmega8A
- Triac gate:
  - `fanPin = PD3` — Triac gate output (gates triac to switch AC supply to fan)
 - Zero-cross sensor: INT0 on PD2 (falling-edge detection)
- IR receiver:
  - `irPin = PB2` — Polling PINB to decode custom SIRC12 pulses.
  - Note: The IR pin uses an internal pull-up in `setup()` for noise immunity; the decoder listens for LOW pulses on PB2.
- Socket & lights:
  - `socketPin = PC5` — Socket ON/OFF
  - `light1Pin = PC0` — Light 1 ON/OFF
  - `light2Pin = PC1` — Light 2 ON/OFF
- 7-segment display (active LOW segments):
  - A: `PD7` (segmentAPin)
  - B: `PB0`
  - C: `PC2`
  - D: `PC3`
  - E: `PC4`
  - F: `PD6`
  - G: `PD5`

Note: The code uses `DDR*` and `PORT*` registers. Use common driving circuits (transistors or ULN driver) for 7-seg and ensure triac gate has a gate resistor and opto-isolator as needed.

---

## 4) Timing & Key Constants ⏱️
- Timer tick & prescaler:
  - Prescaler 64 @ 16 MHz => 4 μs per Timer1 tick (`TIMER_TICK_US = 4`)
- Gate pulse width:
  - `GATE_PULSE_US = 200` (triac gate ON width)
 - Infrared timing constants (SIRC-like 12-bit parser):
   - `IR_LEADER_US = 2300`, `IR_BIT_US = 900`
   - `IR_LEADER_TIMEOUT_US = 55000`, `IR_BIT_TIMEOUT_US = 5000`
- Fan phase delays (microseconds):
- Level->delay mapping is derived from a percent->delay lookup table (`DELAY_FROM_PERCENT`) for deterministic timing.
- The firmware maps levels 1..9 to an effective conduction percent, where level 1 is the minimum configured conduction (`eeprom_min_percent`) and level 9 is approximately 100%.
  - Mapping rule (implemented in code): Ptarget = Pmin + (((level - 1) * (100 - Pmin) + 4) / 8). The `+4` is used to approximate rounding before the integer division; this yields 9 discrete conduction levels from `Pmin` .. `100%`.
- Min/Max Delay:
  - `MIN_DELAY_US = 50`
  - `MAX_DELAY_US = 9500` (cap)
- Watchdog and diagnostics:
  - `WATCHDOG_TIMEOUT_MS = 100` — lastFire timeout check
  - `WATCHDOG_CHECK_INTERVAL_MS = 500`
  - `ZC_FREQ_CHECK_INTERVAL_MS = 1000` (ZC frequency checks; interval changed to 1s window)
  - `ZC_FREQ_MIN = 95` and `ZC_FREQ_MAX = 105` (Hz checkpoint threshold)
  - `SPONTANEOUS_RECOVERY_WINDOW_US = 150000` (150ms): if ZC returns within this window after a watchdog disable, firmware will spontaneously resume fan without waiting full recovery timers.
  - `IR_COMMAND_DEBOUNCE_MS = 120` — Debounce to avoid processing repeated IR commands.

---

## 5) Zero-cross detection & triac gating (flow) 🔁
- Zero-cross detection: INT0 configured to detect falling edge (PD2).
- When zero-cross interrupt triggers (`ISR(INT0_vect)`):
  - Debounce check: ignore if triggered within `ZC_DEBOUNCE_US = 2000` μs of previous.
  - Update `lastZC` and `zcCount` for diagnostics.
  - Toggle `zcParity` each zero-cross (half-cycle parity); the firmware records `scheduledParity` for each scheduled fire and counts per-parity fires for diagnostics.
  - If `fireEnabled` and no timer active:
    - Compute `ticks = targetDelayTicks` (converted from microseconds to timer ticks).
    - Ensure `ticks >= MIN_DELAY_TICKS` and `ticks <= MAX_DELAY_TICKS`.
    - Configure Timer1 (CTC mode) with compare value `OCR1A = ticks`, start Timer1 (CS11|CS10 -> prescaler 64).
    - Enable OCIE1A (compare interrupt) and set `timerActive = true`.
  - When `ticks > MAX_DELAY_TICKS` => no firing is performed (fan at 0% or beyond allowed range).

---

## 6) Timer operation & ISRs ⚙️
- Timer1 compare (`ISR(TIMER1_COMPA_vect)`):
  - On first compare (triacPulseActive=false):
    - Set `PORTD` pin for fan gate high -> turn ON triac gate.
    - Mark `triacPulseActive = true`.
    - Set new OCR1A to gateTicks (calculated = `GATE_PULSE_US`/4us).
  - On second compare:
    - Clear triac gate port (turn gate OFF).
    - Record `lastFire = micros()`.
    - Stop Timer1 (clear TCCR1B), clear `timerActive`.
- Atomic vs volatile:
  - `targetDelayTicks`, `lastFire`, `zcCount` etc. are volatile and accessed atomically using helper functions `atomicReadUL`, `atomicWriteUL`, `atomicReadUI`, `atomicWriteUI`.
  - Helper functions like `atomicReadUL`/`atomicWriteUL` are provided to safely access 32-bit volatile values without disabling interrupts for extended periods.

---

## 7) Controls, IR & Serial Commands 📡
- IR detection:
  - `getSIRC12()` listens for a 12-bit SIRC-like code using `pulseInPB2LOW()`.
  - `validCodes` array: {0xF01,0xF02,0xF03,0xF04,0xF05,0xF06,0xF07} mapped to functions.
- Mapping:
  - IR 0xF01 — Power toggle -> `handlePowerSwitch()`
  - IR 0xF02 — Fan toggle -> `handleFanSwitch()`
  - IR 0xF03 — Fan + (increase) -> `handleFanPlus()`
  - IR 0xF04 — Fan - (decrease) -> `handleFanMinus()`
  - IR 0xF05 — Light 1 toggle -> `handleLight1Switch()`
  - IR 0xF06 — Light 2 toggle -> `handleLight2Switch()`
  - IR 0xF07 — Socket toggle -> `handleSocketSwitch()`
- Software switches used for IR button software events:
  - `PowerSwState`, `FanSwState`, `PlusSwState`, `MinusSwState`, `Light1SwState`, `Light2SwState`, `SocketSwState` default HIGH and set LOW on IR match.
- Serial Commands:
  - 'a' - Power toggle
  - 'b' - Fan toggle
  - 'c' - Fan plus
  - 'd' - Fan minus
  - 'e' - Light 1
  - 'f' - Light 2
  - 'g' - Socket
  - 's' - Print Status (always available). Prints a human-friendly status including:
    - Firmware, Power (ON/OFF), Fan state (ON/OFF) and level
    - Target delay in timer ticks and microseconds
    - Fire (enabled/disabled) and Timer (active/inactive) flags
    - Plug (socket), Light1 and Light2 ON/OFF states
    - Main frequency in Hz and zero-cross triggers per second (ZC/s)
    - `MINP` configured minimum conduction percent
    - 'MINP?' - Query the minimum conduction percent for fan level 1 (0..100).
    - 'MINP=NN' - Set and persist the minimum conduction percent for fan level 1 (NN between 0 and 100).
  - 'k' - removed (previously toggled a zero-cross simulation for testing)
  - Note: Diagnostic toggling (code-based debug toggle) has been removed; diagnostics are always enabled.
  - Note: The IR `pulseInPB2LOW()` implementation is blocking; for production/real-time responsiveness consider moving IR decoding to a timer/interrupt-based decoder.
  - Note: Fan level changes triggered by IR (or serial) are usually scheduled and applied after a short debounce (`IR_COMMAND_DEBOUNCE_MS`) to avoid transients and reduce spikes when buttons are pressed rapidly.
- UI & UX:
  - `handlePowerSwitch`, `handleFanSwitch`, `handleFanPlus`, `handleFanMinus` contain actual state changes and EEPROM updates.

---

## 8) 7-Segment Display Mapping 🔢
- Segments are active LOW.
- `digitSegments[]` contains codes for 0-9, off, and 'F' as index 10/11.
- Calls:
  - `display(byte digitIndex)` sets the 7-seg to display a digit or off.
  - `setFanDisplay(byte fanLevelEEPROM)` updates `targetDelayTicks`, sets `fireEnabled`, and updates `display` to show fan level.
  - Note: The display code iterates a `segmentBitMasks[]` PROGMEM array and builds port masks to write to `PORTD`, `PORTB`, and `PORTC` using `ATOMIC_BLOCK()` for atomic read-modify-write operations; segments are active-LOW.

---

## 9) EEPROM & Persistent States 💾
- EEPROM addresses:
  - `power_addr = 6`, `fan_addr = 7`, `fan_lvl = 8`, `light1_addr = 9`, `light2_addr = 10`, `socket_addr = 11`.
- State Values:
  - `STATE_OFF = 1`, `STATE_ON = 2`.
- On `setup()`:
  - EEPROM read and sanitize values.
  - The minimum conduction percent (EEPROM `fan_min_pct_addr=12`) is validated; if it contains `0xFF` or a value outside `0..100`, it will be set to a default of `5%` and persisted.
  - If power OFF -> set outputs accordingly.
  - Else read and restore fan state & level, lights & sockets.
- `EEPROM.update()` is used to minimize writes.

---

## 10) Diagnostics, Watchdog & Auto-Recovery ⚙️
  - If `fireEnabled` and `micros() - lastFire > WATCHDOG_TIMEOUT_MS * 1000` -> zero-cross signal loss is inferred; then `fireEnabled=false`, `wasDisabledByWatchdog = true`, and recovery mechanisms start.
  - If `AUTO_RECOVERY_ENABLED` and `wasDisabledByWatchdog` is true, attempt to call `setFanDisplay(lastValidLevel)` after `AUTO_RECOVERY_DELAY_SEC` (30s default), up to `MAX_RECOVERY_ATTEMPTS=3`.
  - If zero-cross resumes quickly (within `SPONTANEOUS_RECOVERY_WINDOW_US`) the firmware attempts a spontaneous resume and re-enables fan firing without waiting for the full auto-recovery timer window.
  - Every `ZC_FREQ_CHECK_INTERVAL_MS`, compute frequency `freq = (zcCount * 1000) / elapsed_ms` (zcHz). `cycles` (Hz) = `freq / 2`.
  - Additionally, a `DIAG` print now shows the last zero-cross delta (`lastZCdelta` in μs) and the instantaneous `zcHz`/`cycles` derived from that interval (helpful for jitter/edge detection debugging).
  - If `freq < ZC_FREQ_MIN or freq > ZC_FREQ_MAX` -> log abnormal ZC frequency.
  - `loop()` prints `DIAG` logs every `DIAG_INTERVAL_MS`.
  - Serial 'k' toggling removed (zero-cross simulation removed in this build).
  - DIAG prints now include instantaneous `lastZCdelta` (μs), `zcHzInst`, `cyclesInst`, and `power` percent (normalized conduction power for the current delay).
  - The firmware also tracks `triacFiresTotal` and `triacFiresParity[0]/triacFiresParity[1]` for diagnostics so you can confirm half-cycle firing counts and detect missed pulses.

---

## 11) Building & Flashing ✈️
- Requirements:
  - AVR toolchain (avr-gcc/avrdude) or Arduino IDE; upload via ISP or USB-serial bootloader for ATmega8.
  - Board must be configured for ATmega8/ATmega8A and 16 MHz clock (F_CPU = 16000000UL).
  - Compile-time checks in the code will produce errors if target and F_CPU do not match.
  - Arduino IDE:
    - Select the board that maps to ATmega8/ATmega8A and set the clock to 16 MHz.
    - If the board is unavailable, consider adding the appropriate core or use an ISP programmer.
  - CLI / Toolchain users:
    - Use avr-gcc to compile and avrdude to flash; set `F_CPU=16000000UL` when building.
    - Example using avr-gcc/avrdude (adjust MCU selection and programmer accordingly):
      - avr-gcc -mmcu=atmega8 -DF_CPU=16000000UL -Os -std=gnu99 -c FV5.0.0/FV5.0.0.ino -o main.o
      - avr-gcc -mmcu=atmega8 -o firmware.elf main.o
      - avr-objcopy -O ihex -R .eeprom firmware.elf firmware.hex
      - avrdude -p m8 -c <programmer> -P <port> -U flash:w:firmware.hex
  - Important:
    - Ensure that the board's fuses and bootloader are configured for a 16 MHz external clock if applicable.
    - For ISP programming, ensure the correct programmer settings and port are used.

---

## 12) Test & Troubleshooting Steps 🧪
1. Bench & safety:
   - Power up isolated from mains. Use a GFCI-protected outlet, fuse and proper earthing.
   - Use a resistive dummy load (lamp) for early tests.
  - Serial logs are enabled at 115200 by default.
2. Serial commands for testing:
  - 's' — prints status (power, fan, level, target ticks, fire flag, timerstate).
  - 'k' — removed (no longer supported in this build).
  - 'b', 'c', 'd' — fan toggle, plus, minus respectively.
  - Note: Fan-level changes via IR or serial may be scheduled for a short debounce window (IR_COMMAND_DEBOUNCE_MS) to avoid transients — if a level change doesn't immediately reflect, check for the short delayed commit.
3. Oscilloscope verification:
   - Probe zero-cross sensor output and triac gate to verify timing.
   - Observe correct delay between zero-cross and gate according to selected fan level.
   - Gate ON for ~200 μs.
4. Common failure scenarios:
   - No triac gate pulses: verify DDRD, fanPin (PD3), interrupts enabled (GICR), zero-cross sensor wiring, INT0 config.
   - No zero-cross detection: confirm sensor circuit (isolated opto or resistor divider + optocoupler), check debounce value.
   - Fan not ramping or stuck: check `targetDelayTicks` updates and Timer1 settings; ensure `fireEnabled==true`.
5. Serial & diag:
   - Use Serial output to observe `DIAG` logs and `STATUS` info.
   - Watch for abnormal ZC frequency messages.
6. Safe re-testing:
   - Flushing EEPROM or adjusting state values: to reset states, write default EEPROM or physically change the `power_state`, `fan_state` via serial or code.

---

## 13) Known Issues & Suggested Fixes 🛠️
- IR Detection:
  - `pulseInPB2LOW()` is a custom blocking loop; it will halt the main loop while waiting for pulses. Consider offloading to pin change interrupts or a hardware capture to avoid blocking for long durations.
- EEPROM Wear:
  - Frequent toggles without `EEPROM.update()` are mitigated, but consider storing fewer parameters or employing wear leveling if frequent writes are expected.
- Triac control:
  - The code assumes mains frequency near 50Hz (ZC half-period ~10ms). If used for 60Hz, the mapping and MAX_DELAY_US may need adjustment.
- Input Debounce:
  - IR & software-debounce are used for zero-cross, but hardware inputs aren't debounced. Add a short software debounce if external push buttons are added.

---

## 14) Future Improvements & Ideas 🚀
- Add hardware ADC feedback for current / RPM or thermal protection.
- Move IR decoding to interrupts / timers for more accurate detection and reduce blocking.
- Support 60 Hz boards by adjusting `MAX_DELAY_US` and fan delays, or dynamically compute based on measured ZC frequency.
- Add persistent logging to EEPROM to track failure events, recovery attempts, and last ZC time.
- Implement a unit-test harness for logic (using host-side test with simulated zc pulses).
- Add a "safe mode" that sets `fireEnabled` false and a UI/command to re-enable (manual override).
- Implement global locking or event queue to avoid race conditions when multiple event sources (IR + Serial + direct IO) change states.

---

## Quick Reference Tables

### Pin mapping
| Function | Port Pin | Notes |
|---|---:|---|
| Triac Gate (Fan) | PD3 | Active high triac gate |
| Zero-cross detect | PD2 (INT0) | Falling edge |
| IR receiver | PB2 | PB2 sampling only |
| Socket | PC5 | Digital out |
| Light1 | PC0 | Digital out |
| Light2 | PC1 | Digital out |
| Segment A..G | PD7, PB0, PC2, PC3, PC4, PD6, PD5 | Active LOW segments |

### EEPROM map
| Name | Address | Value |
|---|---:|---|
| Power state | 6 | `STATE_OFF=1`, `STATE_ON=2` |
| Fan state | 7 | `STATE_OFF=1`, `STATE_ON=2` |
| Fan level | 8 | 1..9 |
| Light1 | 9 | `STATE_OFF`/`STATE_ON` |
| Light2 | 10 | `STATE_OFF`/`STATE_ON` |
| Socket | 11 | `STATE_OFF`/`STATE_ON` |
| Min fan % | 12 | 0..100 (default: 5) |

### Fan levels vs Delay (derived)
These values are derived from a percent-to-delay lookup and are shown here for convenience; the runtime mapping uses `DELAY_FROM_PERCENT`.
Note: `FAN_DELAY_US[]` is retained in the `tools/scripts/` output as a generated artifact and is no longer used in the runtime firmware.
Development tools: See `tools/scripts/compute_fan_delays.py` to modify or re-generate the delay lookup values.
| Level | Effective delay (μs) |
|---:|---:|
| 1 | 7980 |
| 2 | 6851 |
| 3 | 6106 |
| 4 | 5472 |
| 5 | 4875 |
| 6 | 4269 |
| 7 | 3601 |
| 8 | 2758 |
| 9 | 50 |

---

## Suggested “How to run” steps (developer & testing)
1. Confirm or set `F_CPU=16000000UL`, compile for `ATmega8` or `ATmega8A`.
2. Build and flash with your chosen loader (for example: `avrdude` or Arduino IDE).
3. Connect to serial (115200) for debug logs (always active).
4. Use an IR remote with codes for 0xF01..0xF07 (custom codes as listed).
5. Use `'s'` to query status; `'b','c','d','e','f','g','a'` to interact with devices; use `MINP?` / `MINP=NN` to query/set minimum fan conduction.
6. Monitor oscilloscope across triac gate and AC line to ensure correct phase angle and proper gate width.

---

## License & Final Notes ⚠️
- License: This project is released under the Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) license. See the `LICENSE` file for full terms. Personal, non-commercial use is allowed; commercial use is prohibited without permission.

## Final Notes & Safety ⚠️
- This code directly controls mains AC line components and assumes knowledge of triac circuits, voltage isolation, and mains safety.
- Always test with safety measures, e.g., GFCI breaker, isolation transformer when possible, and with resistive loads first.
- For CAT I/II/III installations, follow local electrical standards and housing for enclosure & user safety.

---

## Next steps
Suggested next improvements:
- Propose code changes to add more robust IR decoding or non-blocking designs, and to further improve diagnostics if needed.
- Add a wiring diagram as a PNG/SVG and include a small build script or example `Makefile` for easier building and testing.

---

To request any of these improvements or to propose additions, please open an issue or submit a pull request.
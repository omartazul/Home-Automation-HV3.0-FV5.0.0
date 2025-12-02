/*
 * Home-Automation HV3.0 — Firmware FV5.0.0 (clean, production-ready baseline)
 * NOTE: HV = hardware version; FV = firmware version
 * Author: Md. Omar Faruk Tazul Islam
 * Date: March 2, 2025
 * Target: ATmega8 / ATmega8A @ 16 MHz (F_CPU = 16000000UL)
 *
 * This firmware implements appliance control with phase-angle triac firing,
 * zero-cross synchronization (INT0), SIRC-12 infrared control, and
 * EEPROM-backed persistent device state. It is written with direct
 * register access for deterministic timing and low overhead.
 */
 
// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2025 Md. Omar Faruk Tazul Islam
// Licensed under the Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)
// Personal, non-commercial use only. Commercial use or redistribution for sale
// is not permitted without prior written permission from the author.
// For commercial licensing requests: please open a 'Commercial license request' issue in this repository.

#include <Arduino.h>
#include <EEPROM.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>

// Firmware version
#define FIRMWARE_VERSION "FV5.0.0"

// ----- STRONG BUILD-TIME SAFEGUARDS -----
#if !defined(__AVR_ATmega8__) && !defined(__AVR_ATmega8A__)
#error "This sketch only supports ATmega8 or ATmega8A targets. Build aborted."
#endif

#ifndef F_CPU
#error "F_CPU must be defined as 16000000UL in project settings."
#endif
#if (F_CPU != 16000000UL)
#error "F_CPU must be 16000000UL (16 MHz) to match timing assumptions."
#endif

// Diagnostics/Logging: macros forward directly to Serial (always enabled in this build).
// Keep log messages compact; avoid expensive operations (floating-point, large buffers).
#define LOG(...) Serial.print(__VA_ARGS__)
#define LOGLN(...) Serial.println(__VA_ARGS__)
// Print strings from flash to save RAM
#define LOGS(x) Serial.print(F(x))
#define LOGLNS(x) Serial.println(F(x))

// ----- CONSTANTS & TIMING -----
const unsigned int GATE_PULSE_US = 200; // Triac gate duration (us)
const unsigned int MIN_DELAY_US = 50;
const unsigned int MAX_DELAY_US = 9500;
// Timer tick at prescaler 64 @16MHz => 4 us per tick
const unsigned int TIMER_TICK_US = 4;
const unsigned int MIN_DELAY_TICKS = (MIN_DELAY_US + TIMER_TICK_US - 1) / TIMER_TICK_US;
const unsigned int MAX_DELAY_TICKS = MAX_DELAY_US / TIMER_TICK_US;
const unsigned int WATCHDOG_TIMEOUT_MS = 100;
const bool AUTO_RECOVERY_ENABLED = true;
const unsigned int AUTO_RECOVERY_DELAY_SEC = 30;
const uint8_t MAX_RECOVERY_ATTEMPTS = 3;
const bool ZC_FREQUENCY_CHECK = true;
const unsigned long SPONTANEOUS_RECOVERY_WINDOW_US = 150000UL;
const unsigned int IR_LEADER_US = 2300;
const unsigned int IR_BIT_US = 900;
const unsigned long IR_LEADER_TIMEOUT_US = 55000UL;
const unsigned long IR_BIT_TIMEOUT_US = 5000UL;
const unsigned int ZC_DEBOUNCE_US = 2000;
const unsigned int WATCHDOG_CHECK_INTERVAL_MS = 500;
const unsigned int ZC_FREQ_CHECK_INTERVAL_MS = 1000; // changed from 2000 to 1000 (1s window)
const unsigned int ZC_FREQ_MIN = 95;
const unsigned int ZC_FREQ_MAX = 105;
const unsigned long DIAG_INTERVAL_MS = 1000UL;
// Half-cycle computation is intentionally omitted; this firmware uses
// a percent->delay lookup table (DELAY_FROM_PERCENT) for deterministic timing.
// Allowable difference between zc-count and triac fires before warning
const unsigned int TRIAC_MISS_TOLERANCE = 2;
// IR command debounce in milliseconds to avoid repeated processing of remote repeats
const unsigned long IR_COMMAND_DEBOUNCE_MS = 120UL;
// Fan levels always map 1=slowest, 9=fastest
// Ramping parameters: this firmware applies level changes immediately (no soft ramping)

// Corresponding approximate conduction power percent for each delay value (index 0 -> 100%)
const uint8_t FAN_POWER_PERCENT[9] PROGMEM = {100, 88, 76, 64, 53, 41, 29, 17, 5};
const uint16_t validCodes[7] PROGMEM = {0xF01, 0xF02, 0xF03, 0xF04, 0xF05, 0xF06, 0xF07};
// Main lookup: percent [0..100] -> delay_us. Values are stored in flash (PROGMEM)
// and are clamped to MIN_DELAY_US / MAX_DELAY_US by the code when applied.
// The CLI command `MINP` adjusts the minimum conduction percent persisted in EEPROM.
const uint16_t DELAY_FROM_PERCENT[101] PROGMEM = {9500, 8840, 8531, 8310, 8132, 7980, 7846, 7724, 7612, 7508, 7411, 7319, 7231, 7147, 7067, 6990, 6915, 6842, 6772, 6704, 6637, 6572, 6508, 6445, 6384, 6324, 6264, 6206, 6149, 6092, 6036, 5980, 5926, 5871, 5818, 5765, 5712, 5659, 5607, 5556, 5504, 5453, 5402, 5351, 5301, 5251, 5200, 5150, 5100, 5050, 5000, 4950, 4900, 4850, 4800, 4749, 4699, 4649, 4598, 4547, 4496, 4444, 4393, 4341, 4288, 4235, 4182, 4129, 4074, 4020, 3964, 3908, 3851, 3794, 3736, 3676, 3616, 3555, 3492, 3428, 3363, 3296, 3228, 3158, 3085, 3010, 2933, 2853, 2769, 2681, 2589, 2492, 2388, 2276, 2154, 2020, 1868, 1690, 1469, 1160, 50};

// ----- PINS - Direct port bit constants for ATmega8/ATmega8A -----
// Output triac gate: PD3; zero-cross: PD2 (INT0)
const byte fanPin = PD3;
const byte socketPin = PC5;
const byte light1Pin = PC0;
const byte light2Pin = PC1;
// Zero-cross detection is via INT0 (PD2), configured directly in the ISR setup
const byte irPin = PB2; // IR receiver on PB2

// 7-segment mapping: uses discrete port bits for segments A..G
const byte segmentAPin = PD7, segmentBPin = PB0, segmentCPin = PC2, segmentDPin = PC3, segmentEPin = PC4, segmentFPin = PD6, segmentGPin = PD5;
// Note: segmentPorts array removed (unused) to save RAM
const byte segmentBitMasks[] PROGMEM = {(1<<7), (1<<0), (1<<2), (1<<3), (1<<4), (1<<6), (1<<5)};

// 0-9, off, 'F' for display
const byte digitSegments[] PROGMEM = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x00, 0x71};

// EEPROM Addresses
const byte power_addr = 6, fan_addr = 7, fan_lvl = 8, light1_addr = 9, light2_addr = 10, socket_addr = 11;

// EEPROM state values
const byte STATE_OFF = 1;
const byte STATE_ON = 2;

// ===== GLOBALS (shared with ISRs) =====
volatile unsigned int targetDelayTicks = 0;
volatile bool fireEnabled = false;
volatile bool timerActive = false;
volatile unsigned long lastZeroCross = 0;
volatile unsigned long lastFire = 0;
volatile unsigned int zcCount = 0;
volatile unsigned long lastZC = 0;
volatile unsigned long lastZCDelta = 0; // delta in microseconds between consecutive zero-cross events (for debug)
volatile bool triacPulseActive = false;
volatile bool zcParity = false; // toggles every INT0 call (half-cycle parity)
volatile bool scheduledParity = false; // parity for the currently scheduled fire
volatile unsigned int triacFiresTotal = 0;
volatile unsigned int triacFiresParity[2] = {0,0};

// Reset triac counters atomically
static inline void resetTriacCounters() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    triacFiresTotal = 0;
    triacFiresParity[0] = 0;
    triacFiresParity[1] = 0;
  }
}

volatile uint8_t eeprom_fan_state = STATE_OFF;
volatile uint8_t eeprom_power_state = STATE_OFF;
volatile uint8_t eeprom_light1_state = STATE_OFF;
volatile uint8_t eeprom_light2_state = STATE_OFF;
volatile uint8_t eeprom_socket_state = STATE_OFF;
volatile uint8_t eeprom_fan_level = 1;
const byte fan_min_pct_addr = 12;
volatile uint8_t eeprom_min_percent = 5; // default 5%
// Pending fan changes: schedule commit in main loop to avoid accidental spikes
volatile byte pendingFanLevel = 0;
volatile byte pendingDisplayDigit = 0;
volatile unsigned long pendingApplyTime = 0UL;

// Software switch state flags (set by IR decode or Serial commands)
// If you add hardware buttons, add debounce logic here for reliable input.
bool PowerSwState = HIGH, LastPowerSwState = HIGH;
bool FanSwState = HIGH, LastFanSwState = HIGH;
bool PlusSwState = HIGH, LastPlusSwState = HIGH;
bool MinusSwState = HIGH, LastMinusSwState = HIGH;
bool Light1SwState = HIGH, LastLight1SwState = HIGH;
bool Light2SwState = HIGH, LastLight2SwState = HIGH;
bool SocketSwState = HIGH, LastSocketSwState = HIGH;

// Diagnostics/test toggles
// Diagnostics: serial-based diagnostic messages are always enabled.
unsigned long lastIRMillis = 0UL;
int lastIRCode = -1;

// Watchdog & auto-recovery helpers
static bool wasDisabledByWatchdog = false;
static unsigned long watchdogDisableTime = 0UL;
static byte lastValidLevel = 1;
static uint8_t recoveryAttempts = 0;

// ===== ATOMIC ACCESS HELPERS =====
static inline unsigned long atomicReadUL(volatile unsigned long &v) {
  unsigned long tmp;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { tmp = v; }
  return tmp;
}
static inline void atomicWriteUL(volatile unsigned long &v, unsigned long val) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = val; }
}
static inline unsigned int atomicReadUI(volatile unsigned int &v) {
  unsigned int tmp;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { tmp = v; }
  return tmp;
}
static inline void atomicWriteUI(volatile unsigned int &v, unsigned int val) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = val; }
}

// Compile-time asserts to ensure arrays are the expected sizes
typedef char Assert_DigitSegments[(sizeof(digitSegments)/sizeof(digitSegments[0]) >= 12) ? 1 : -1];

// ===== PROTOTYPES =====
// Polls IR receiver; sets software switch flags if a valid code was detected
void IR_Detect();
// Returns a valid SIRC-12 code or -1 if none detected
int getSIRC12();
// Pulse input reader for PB2: returns LOW pulse microseconds (blocking)
unsigned long pulseInPB2LOW(unsigned long timeout);
// Display a digit (0..9 or special) on the 7-segment display (active LOW mapping)
void display(byte digitIndex);
// Update target delay for a requested fan level and reflect on display
void setFanDisplay(byte fanLevelEEPROM);
// UI handlers for toggling power/fan/lights/socket
void handlePowerSwitch();
void handleFanSwitch();
void handleFanPlus();
void handleFanMinus();
// Set the fan level and commit to EEPROM and timer
void setFanLevel(byte levelEEPROM, byte displayDigit);
void handleLight1Switch();
void handleLight2Switch();
void handleSocketSwitch();
// Toggle output helper that updates port and persists to EEPROM
void toggleOutput(byte addr, byte pin, volatile uint8_t &port, volatile uint8_t &cachedState);

// ===== SETUP & LOOP =====
void setup() {
  // Explicitly configure only ATmega8/ATmega8A hardware registers needed
  // Configure outputs
  DDRD |= (1 << fanPin) | (1 << segmentAPin) | (1 << segmentFPin) | (1 << segmentGPin);
  DDRC |= (1 << socketPin) | (1 << light1Pin) | (1 << light2Pin) | (1 << segmentCPin) | (1 << segmentDPin) | (1 << segmentEPin);
  DDRB |= (1 << segmentBPin);
  // Set ir pin as input with pull-up for noise immunity
  DDRB &= ~(1 << irPin);
  PORTB |= (1 << irPin);  // Enable internal pull-up on IR receiver

  // Initialize output pins to safe off states
  PORTD &= ~(1 << fanPin);  // Triac gate off (LOW)

  // Always start Serial (debugging always enabled)
  Serial.begin(115200);
  LOGS("FIRMWARE="); LOGS(FIRMWARE_VERSION); LOGLN("");

  // Configure INT0 for falling-edge zero-cross detection (CT354A typical)
  MCUCR &= ~(1 << ISC00);
  MCUCR |= (1 << ISC01);
  GICR |= (1 << INT0);
  sei();

  // Timer1 default disabled (we enable it only in ISR for gate events)
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK &= ~(1 << OCIE1A);

  // Read and sanitize EEPROM-backed states
  byte powerState = EEPROM.read(power_addr);
  if (powerState != STATE_OFF && powerState != STATE_ON) powerState = STATE_OFF;
  eeprom_power_state = powerState;
  eeprom_fan_state = EEPROM.read(fan_addr);
  if (eeprom_fan_state != STATE_OFF && eeprom_fan_state != STATE_ON) eeprom_fan_state = STATE_OFF;
  eeprom_fan_level = EEPROM.read(fan_lvl);
  if (eeprom_fan_level < 1 || eeprom_fan_level > 9) eeprom_fan_level = 1;
  eeprom_min_percent = EEPROM.read(fan_min_pct_addr);
  if (eeprom_min_percent > 100 || eeprom_min_percent == 0xFF) {
    eeprom_min_percent = 5; // default: 5% if not set
    EEPROM.update(fan_min_pct_addr, eeprom_min_percent);
  }
  // Log configured minimum percent for clarity
  LOGS("MINP="); LOG((int)eeprom_min_percent); LOGLN("");
  eeprom_light1_state = EEPROM.read(light1_addr);
  if (eeprom_light1_state != STATE_OFF && eeprom_light1_state != STATE_ON) eeprom_light1_state = STATE_OFF;
  eeprom_light2_state = EEPROM.read(light2_addr);
  if (eeprom_light2_state != STATE_OFF && eeprom_light2_state != STATE_ON) eeprom_light2_state = STATE_OFF;
  eeprom_socket_state = EEPROM.read(socket_addr);
  if (eeprom_socket_state != STATE_OFF && eeprom_socket_state != STATE_ON) eeprom_socket_state = STATE_OFF;

  // Initialize hardware to persisted states
  if (eeprom_power_state == STATE_OFF) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { PORTC &= ~((1 << socketPin) | (1 << light1Pin) | (1 << light2Pin)); }
    display(10);
    fireEnabled = false;
    atomicWriteUI(targetDelayTicks, MAX_DELAY_TICKS + 1);
  } else {
    if (eeprom_fan_state == STATE_OFF) display(11);
    else if (eeprom_fan_state == STATE_ON) setFanDisplay(eeprom_fan_level);
    byte portCState = PORTC;
    if (eeprom_light1_state == STATE_ON) portCState |= (1 << light1Pin); else portCState &= ~(1 << light1Pin);
    if (eeprom_light2_state == STATE_ON) portCState |= (1 << light2Pin); else portCState &= ~(1 << light2Pin);
    if (eeprom_socket_state == STATE_ON) portCState |= (1 << socketPin); else portCState &= ~(1 << socketPin);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { PORTC = portCState; }
    // Diagnostic logging is always on; no runtime toggle.
  }
}

void loop() {
  // Reset IR-driven software button states each loop
  PowerSwState = HIGH; FanSwState = HIGH; PlusSwState = HIGH; MinusSwState = HIGH; Light1SwState = HIGH; Light2SwState = HIGH; SocketSwState = HIGH;
  IR_Detect();

  // apply pending scheduled fan change when timed
  unsigned long pendingTime = atomicReadUL(pendingApplyTime);
  if (pendingTime != 0 && (millis() >= pendingTime)) {
    atomicWriteUL(pendingApplyTime, 0);
    // commit pending fan change
    setFanLevel(pendingFanLevel, pendingDisplayDigit);
    // consumed
    pendingFanLevel = 0;
  }

  static char atBuf[32];
  static byte atPos = 0;
  static bool atCollecting = false;
  bool serialDataAvailable = (Serial.available() > 0);
  char data = 0;
  if (serialDataAvailable) {
    data = Serial.read();
    // MINP command parsing: start with uppercase 'M' (lowercase 'm' accepted)
    if (atCollecting) {
      if (data == '\r' || data == '\n') {
        atBuf[atPos] = '\0';
        // process AT command
        if (strncmp(atBuf, "MINP?", 5) == 0) {
          LOGS("MINP="); LOG((int)eeprom_min_percent); LOGLN("");
        } else if (strncmp(atBuf, "MINP=", 5) == 0) {
          int v = atoi(atBuf + 5);
          if (v < 0) v = 0; if (v > 100) v = 100;
          EEPROM.update(fan_min_pct_addr, (uint8_t)v);
          eeprom_min_percent = (uint8_t)v;
          LOGS("MINP set "); LOG(v); LOGLN("");
          // Recalculate current ticks if fan is on
          if (eeprom_fan_state == STATE_ON && eeprom_power_state == STATE_ON) {
            setFanDisplay(eeprom_fan_level);
          }
        }
        atCollecting = false; atPos = 0;
      } else {
        // Normalize to uppercase for easier parsing
        if ((data >= 'a') && (data <= 'z')) data = (char)(data - ('a' - 'A'));
        if (atPos < sizeof(atBuf)-1) {
          atBuf[atPos++] = data;
        } else {
          // Buffer overflow, reset collection and notify
          atCollecting = false; atPos = 0;
          LOGLNS("ERR: MINP command too long");
        }
      }
    } else if (data == 'M' || data == 'm') {
      if ((data >= 'a') && (data <= 'z')) data = (char)(data - ('a' - 'A'));
      atCollecting = true; atPos = 0; atBuf[atPos++] = data;
    }
  }

  if (serialDataAvailable || (PowerSwState != LastPowerSwState) || (FanSwState != LastFanSwState) || (PlusSwState != LastPlusSwState) || (MinusSwState != LastMinusSwState) || (SocketSwState != LastSocketSwState) || (Light1SwState != LastLight1SwState) || (Light2SwState != LastLight2SwState)) {
    if ((data == 'a') || (PowerSwState == LOW)) handlePowerSwitch();
    else if ((data == 'b') || (FanSwState == LOW)) handleFanSwitch();
    else if ((data == 'c') || (PlusSwState == LOW)) handleFanPlus();
    else if ((data == 'd') || (MinusSwState == LOW)) handleFanMinus();
    else if ((data == 'e') || (Light1SwState == LOW)) handleLight1Switch();
    else if ((data == 'f') || (Light2SwState == LOW)) handleLight2Switch();
    else if ((data == 'g') || (SocketSwState == LOW)) handleSocketSwitch();
    else if (serialDataAvailable && (data == 's')) {
      unsigned int tTicks = atomicReadUI(targetDelayTicks);
      unsigned long lastZCDeltaLocal = atomicReadUL(lastZCDelta);
      unsigned long zcInstHz = (lastZCDeltaLocal > 0) ? (1000000UL / lastZCDeltaLocal) : 0UL; // zero-cross events per second
      unsigned long mainsHz = zcInstHz / 2UL; // mains cycles per second (50 or 60 Hz typical)
      // Human-readable states
      const char *powerStr = (eeprom_power_state == STATE_ON) ? "ON" : "OFF";
      const char *fanStr = (eeprom_fan_state == STATE_ON) ? "ON" : "OFF";
      const char *plugStr = (eeprom_socket_state == STATE_ON) ? "ON" : "OFF";
      const char *light1Str = (eeprom_light1_state == STATE_ON) ? "ON" : "OFF";
      const char *light2Str = (eeprom_light2_state == STATE_ON) ? "ON" : "OFF";
      const char *fireStr = fireEnabled ? "ENABLED" : "DISABLED";
      const char *timerStr = timerActive ? "ACTIVE" : "INACTIVE";
      // Print improved status line
      LOGS("STATUS: FW="); LOGS(FIRMWARE_VERSION);
      LOGS(" | Power="); LOG(powerStr);
      LOGS(" | Fan="); LOG(fanStr); LOGS(" (L="); LOG((int)eeprom_fan_level); LOGS(")");
      LOGS(" | Delay="); LOG(tTicks); LOGS(" ticks ("); LOG((long)(tTicks * TIMER_TICK_US)); LOGS(" us)");
      LOGS(" | Fire="); LOG(fireStr); LOGS(" | Timer="); LOG(timerStr);
      LOGS(" | Plug="); LOG(plugStr);
      LOGS(" | Light1="); LOG(light1Str); LOGS(" | Light2="); LOG(light2Str);
      LOGS(" | MainHz="); LOG(mainsHz); LOGS(" | ZC/s="); LOG(zcInstHz);
      LOGS(" | MINP="); LOG((int)eeprom_min_percent);
      LOGLN("");
    }
    // Diagnostics are printed to the serial console (always enabled in this build).
    }

  LastPowerSwState = PowerSwState; LastFanSwState = FanSwState; LastPlusSwState = PlusSwState; LastMinusSwState = MinusSwState; LastSocketSwState = SocketSwState; LastLight1SwState = Light1SwState; LastLight2SwState = Light2SwState;

  // Watchdog-like frequency check for zero-cross & fire
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > WATCHDOG_CHECK_INTERVAL_MS) {
    lastCheck = millis();
    unsigned long lastFireCopy = atomicReadUL(lastFire);
    if (fireEnabled && (micros() - lastFireCopy > (WATCHDOG_TIMEOUT_MS * 1000UL))) {
      LOGLNS("ERR: ZC LOST!");
      fireEnabled = false;
      wasDisabledByWatchdog = true;
      watchdogDisableTime = millis();
      recoveryAttempts = 0;
      resetTriacCounters();
    } else if (wasDisabledByWatchdog && !fireEnabled) {
      unsigned long lastZCCopy = atomicReadUL(lastZC);
      unsigned long zcAge = micros() - lastZCCopy;
      if (zcAge < SPONTANEOUS_RECOVERY_WINDOW_US && eeprom_power_state == STATE_ON && eeprom_fan_state == STATE_ON) {
        LOGLNS("ZC OK: resume fan");
        wasDisabledByWatchdog = false;
        recoveryAttempts = 0;
        setFanDisplay(lastValidLevel);
      }
    }
  }

  // Auto-recovery attempts
  if (AUTO_RECOVERY_ENABLED && wasDisabledByWatchdog && eeprom_power_state == STATE_ON && eeprom_fan_state == STATE_ON) {
    if ((millis() - watchdogDisableTime) > (AUTO_RECOVERY_DELAY_SEC * 1000UL)) {
      if (recoveryAttempts < MAX_RECOVERY_ATTEMPTS) {
        LOGS("RECOVERY:"); LOG(recoveryAttempts + 1); LOGLN("/3");
        setFanDisplay(lastValidLevel);
        recoveryAttempts++;
        watchdogDisableTime = millis();
      } else if (recoveryAttempts == MAX_RECOVERY_ATTEMPTS) {
        LOGLNS("REC MAX");
        recoveryAttempts++;
      }
    }
  }
}

// ===== IR detection & parsing =====
// WARNING: IR decoding is blocking and may delay main loop responsiveness.
// For production, consider moving IR decoding to interrupts or timer-based logic.
void IR_Detect() {
  int Code = getSIRC12();
  if (Code != -1) {
    unsigned long msNow = millis();
    // ignore immediate repeats of the same code
    if ((Code == lastIRCode) && ((msNow - lastIRMillis) < IR_COMMAND_DEBOUNCE_MS)) {
      return;
    }
    lastIRCode = Code;
    lastIRMillis = msNow;
    LOGS("IR:"); LOG(Code, HEX); LOGLN("");
    if (Code == 0xF01) PowerSwState = LOW;
    else if (Code == 0xF02) FanSwState = LOW;
    else if (Code == 0xF03) PlusSwState = LOW;
    else if (Code == 0xF04) MinusSwState = LOW;
    else if (Code == 0xF05) Light1SwState = LOW;
    else if (Code == 0xF06) Light2SwState = LOW;
    else if (Code == 0xF07) SocketSwState = LOW;
  }
}

int getSIRC12() {
  if (pulseInPB2LOW(IR_LEADER_TIMEOUT_US) > IR_LEADER_US) {
    int IRCode = 0;
    for (int i = 0; i < 12; ++i) IRCode |= (pulseInPB2LOW(IR_BIT_TIMEOUT_US) > IR_BIT_US) << i;
    for (int j = 0; j < 7; ++j) if (IRCode == pgm_read_word_near(validCodes + j)) return IRCode;
  }
  return -1;
}

// Custom pulseIn for PB2 LOW-only for SIRC reading (standardized for robustness).
unsigned long pulseInPB2LOW(unsigned long timeout) {
  uint8_t irMask = (1 << irPin);
  unsigned long startWait = micros();
  // Wait for start of LOW pulse (HIGH to LOW transition), with overall timeout
  while ((micros() - startWait) < timeout) {
    if (!(PINB & irMask)) break;  // Exit when LOW starts
  }
  if ((micros() - startWait) >= timeout) return 0;  // Timed out waiting for LOW start

  // Now time the LOW duration
  unsigned long pulseStart = micros();
  unsigned long elapsed;
  while ((elapsed = micros() - pulseStart) < timeout) {
    if (PINB & irMask) return elapsed;  // HIGH detected: end of LOW
  }
  return elapsed;  // Timeout during LOW: return partial duration
}

// ===== ISRs =====
ISR(INT0_vect) {
  unsigned long now = micros();
  // NOTE: zero-cross detection is handled here; no simulation code is included
  unsigned long prevZC = lastZC;
  if ((now - lastZeroCross) < ZC_DEBOUNCE_US) return;
  lastZeroCross = now;
  lastZC = now;
  if (prevZC != 0) lastZCDelta = now - prevZC; else lastZCDelta = 0;
  zcCount++;
  // toggle parity on each zero cross; helps to count per half cycle
  zcParity = !zcParity;
  if (!fireEnabled || timerActive) return;
  // Direct access is safe here - we're in ISR with interrupts disabled
  unsigned int ticks = targetDelayTicks;
  if (ticks < MIN_DELAY_TICKS) ticks = MIN_DELAY_TICKS;
  if (ticks > MAX_DELAY_TICKS) return;
  // Direct register access - no ATOMIC_BLOCK needed in ISR
  TCNT1 = 0;
  OCR1A = ticks;
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC, prescaler 64
  TIMSK |= (1 << OCIE1A);
  timerActive = true;
  triacPulseActive = false;
  // record the parity the scheduled fire will happen on
  scheduledParity = zcParity;
}

ISR(TIMER1_COMPA_vect) {
  if (!triacPulseActive) {
    if (fireEnabled) {
      PORTD |= (1 << fanPin);
      triacPulseActive = true;
      unsigned int gateTicks = (GATE_PULSE_US + TIMER_TICK_US - 1) / TIMER_TICK_US;
      // Direct register access - no ATOMIC_BLOCK needed in ISR
      OCR1A = gateTicks;
      // record the triac pulse (count & parity)
      triacFiresTotal++;
      if (scheduledParity) triacFiresParity[1]++; else triacFiresParity[0]++;
      return;
    } else {
      TCCR1B = 0;
      timerActive = false;
      triacPulseActive = false;
      return;
    }
  } else {
    PORTD &= ~(1 << fanPin);
    lastFire = micros();
    triacPulseActive = false;
    // Direct register access - no ATOMIC_BLOCK needed in ISR
    TCCR1B = 0;
    timerActive = false;
    return;
  }
}

// ===== UTILITY HELPERS =====
void display(byte digitIndex) {
  if (digitIndex > 11) digitIndex = 10;
  byte segments = pgm_read_byte_near(digitSegments + digitIndex);
  // Prepare masks for each port that will be updated (active-LOW segments)
  uint8_t portD_set = 0, portD_clear = 0;
  uint8_t portB_set = 0, portB_clear = 0;
  uint8_t portC_set = 0, portC_clear = 0;
  for (int i = 0; i < 7; ++i) {
    byte mask = pgm_read_byte_near(segmentBitMasks + i);
    // if segment bit is 1 -> segment ON -> port bit should be driven LOW (clear)
    if (segments & (1 << i)) {
      switch (i) {
        case 0: portD_clear |= mask; break; // A -> PORTD
        case 1: portB_clear |= mask; break; // B -> PORTB
        case 2: portC_clear |= mask; break; // C -> PORTC
        case 3: portC_clear |= mask; break; // D -> PORTC
        case 4: portC_clear |= mask; break; // E -> PORTC
        case 5: portD_clear |= mask; break; // F -> PORTD
        case 6: portD_clear |= mask; break; // G -> PORTD
      }
    } else {
      // segment OFF -> port bit HIGH (set)
      switch (i) {
        case 0: portD_set |= mask; break;
        case 1: portB_set |= mask; break;
        case 2: portC_set |= mask; break;
        case 3: portC_set |= mask; break;
        case 4: portC_set |= mask; break;
        case 5: portD_set |= mask; break;
        case 6: portD_set |= mask; break;
      }
    }
  }
  // Perform read-modify-write for each port in a single small ATOMIC block each
  // For active-LOW: clear mask turns segment ON (drive LOW), set mask turns segment OFF (drive HIGH)
  // Apply: first clear the bits we want LOW, then set the bits we want HIGH
  // Combined mask must handle both: clear the "clear" bits AND set the "set" bits
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    uint8_t allMaskD = portD_clear | portD_set;
    uint8_t allMaskB = portB_clear | portB_set;
    uint8_t allMaskC = portC_clear | portC_set;
    // newValue = (current & ~allMask) | portX_set  => clears all segment bits, then sets the ones that should be HIGH
    uint8_t curD = PORTD;
    uint8_t newD = (curD & ~allMaskD) | portD_set;
    if (newD != curD) PORTD = newD;
    uint8_t curB = PORTB;
    uint8_t newB = (curB & ~allMaskB) | portB_set;
    if (newB != curB) PORTB = newB;
    uint8_t curC = PORTC;
    uint8_t newC = (curC & ~allMaskC) | portC_set;
    if (newC != curC) PORTC = newC;
  }
}

void setFanDisplay(byte fanLevelEEPROM) {
  byte level = (fanLevelEEPROM < 1) ? 1 : (fanLevelEEPROM);
  if (level > 9) level = 9;
  // compute Ptarget percentage for this level: linear from min_percent to 100 across 8 steps
  byte Pmin = eeprom_min_percent;
  unsigned int Ptarget = (unsigned int)Pmin + ((unsigned int)(level - 1) * (100 - Pmin) + 4) / 8;
  if (Ptarget > 100) Ptarget = 100;
  unsigned int delay_us = pgm_read_word_near(DELAY_FROM_PERCENT + Ptarget);
  unsigned int ticks = (delay_us + TIMER_TICK_US - 1) / TIMER_TICK_US;
  if (ticks < MIN_DELAY_TICKS) ticks = MIN_DELAY_TICKS;
  if (ticks > MAX_DELAY_TICKS) ticks = MAX_DELAY_TICKS;
  // minimize atomic time: set timing flags first, update display outside
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    targetDelayTicks = ticks;  // Direct assignment inside ATOMIC_BLOCK
    fireEnabled = true;
    lastValidLevel = level;
  }
  display(level);
}

// ===== Event handlers =====
void handlePowerSwitch() {
  if (eeprom_power_state == STATE_OFF) {
    if (eeprom_fan_state == STATE_OFF) display(11);
    else if (eeprom_fan_state == STATE_ON) setFanDisplay(eeprom_fan_level);
    byte portCState = PORTC;
    if (eeprom_light1_state == STATE_ON) portCState |= (1 << light1Pin); else portCState &= ~(1 << light1Pin);
    if (eeprom_light2_state == STATE_ON) portCState |= (1 << light2Pin); else portCState &= ~(1 << light2Pin);
    if (eeprom_socket_state == STATE_ON) portCState |= (1 << socketPin); else portCState &= ~(1 << socketPin);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      PORTC = portCState;
      PORTD &= ~(1 << fanPin);  // Ensure triac off
    }
    EEPROM.update(power_addr, STATE_ON);
    eeprom_power_state = STATE_ON;
    // Reset counters on power on (target already set by setFanDisplay if fan on)
    resetTriacCounters();
  } else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      PORTC &= ~((1 << socketPin) | (1 << light1Pin) | (1 << light2Pin));
      PORTD &= ~(1 << fanPin);  // Ensure triac off
    }
    display(10);
    EEPROM.update(power_addr, STATE_OFF);
    eeprom_power_state = STATE_OFF;
    fireEnabled = false;
    atomicWriteUI(targetDelayTicks, MAX_DELAY_TICKS + 1);
    resetTriacCounters();
  }
}

void handleFanSwitch() {
  if ((eeprom_fan_state == STATE_OFF) && (eeprom_power_state == STATE_ON)) {
    // schedule startup instead of immediate apply to avoid transients
    pendingFanLevel = eeprom_fan_level;
    pendingDisplayDigit = eeprom_fan_level;
    atomicWriteUL(pendingApplyTime, millis() + IR_COMMAND_DEBOUNCE_MS);
    display(pendingDisplayDigit);
    EEPROM.update(fan_addr, STATE_ON);
    eeprom_fan_state = STATE_ON;
  } else if ((eeprom_fan_state == STATE_ON) && (eeprom_power_state == STATE_ON)) {
    display(11);
    EEPROM.update(fan_addr, STATE_OFF);
    eeprom_fan_state = STATE_OFF;
    fireEnabled = false;
    atomicWriteUI(targetDelayTicks, MAX_DELAY_TICKS + 1);
    resetTriacCounters();
  }
}

void handleFanPlus() {
  if ((eeprom_fan_state == STATE_ON) && (eeprom_power_state == STATE_ON)) {
    byte currentLevel = eeprom_fan_level;
    byte index = currentLevel - 1;
    if (index < 8) {
      byte newIndex = index + 1;
      // schedule the change to reduce transient spikes if user presses fast
      pendingFanLevel = newIndex + 1;
      pendingDisplayDigit = newIndex + 1;
      atomicWriteUL(pendingApplyTime, millis() + IR_COMMAND_DEBOUNCE_MS);
      display(pendingDisplayDigit);
    }
  }
}

void handleFanMinus() {
  if ((eeprom_fan_state == STATE_ON) && (eeprom_power_state == STATE_ON)) {
    byte currentLevel = eeprom_fan_level;
    byte index = currentLevel - 1;
    if (index > 0) {
      byte newIndex = index - 1;
      // schedule the change to reduce transient spikes if user presses fast
      pendingFanLevel = newIndex + 1;
      pendingDisplayDigit = newIndex + 1;
      atomicWriteUL(pendingApplyTime, millis() + IR_COMMAND_DEBOUNCE_MS);
      display(pendingDisplayDigit);
    }
  }
}

void setFanLevel(byte levelEEPROM, byte displayDigit) {
  byte level = (levelEEPROM < 1) ? 1 : (levelEEPROM);
  if (level > 9) level = 9;
  byte Pmin = eeprom_min_percent;
  unsigned int Ptarget = (unsigned int)Pmin + ((unsigned int)(level - 1) * (100 - Pmin) + 4) / 8;
  if (Ptarget > 100) Ptarget = 100;
  unsigned int delay_us = pgm_read_word_near(DELAY_FROM_PERCENT + Ptarget);
  unsigned int ticks = (delay_us + TIMER_TICK_US - 1) / TIMER_TICK_US;
  if (ticks < MIN_DELAY_TICKS) ticks = MIN_DELAY_TICKS;
  if (ticks > MAX_DELAY_TICKS) ticks = MAX_DELAY_TICKS;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // commit new target ticks and update volatile variables atomically
    targetDelayTicks = ticks;  // Direct assignment inside ATOMIC_BLOCK
    eeprom_fan_level = level;
    lastValidLevel = level;
    fireEnabled = true;
  }
  // persist the sanitized level without blocking interrupts
  EEPROM.update(fan_lvl, level);
  // display selection: if there is a requested displayDigit, prefer that (sanitized)
  if (displayDigit <= 11) display(displayDigit); else display(level);
}

void handleLight1Switch() { if (eeprom_power_state == STATE_ON) toggleOutput(light1_addr, light1Pin, PORTC, eeprom_light1_state); }
void handleLight2Switch() { if (eeprom_power_state == STATE_ON) toggleOutput(light2_addr, light2Pin, PORTC, eeprom_light2_state); }
void handleSocketSwitch() { if (eeprom_power_state == STATE_ON) toggleOutput(socket_addr, socketPin, PORTC, eeprom_socket_state); }

void toggleOutput(byte addr, byte pin, volatile uint8_t &port, volatile uint8_t &cachedState) {
  uint8_t newState;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (cachedState == STATE_OFF) {
      port |= (1 << pin);
      cachedState = STATE_ON;
      newState = STATE_ON;
    } else {
      port &= ~(1 << pin);
      cachedState = STATE_OFF;
      newState = STATE_OFF;
    }
  }
  // Persist the change outside the ATOMIC_BLOCK to avoid long interrupt disable
  EEPROM.update(addr, newState);
}

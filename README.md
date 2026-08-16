# Voice-Controlled UAV System — ESP32, ESP-NOW and CRSF

## Overview

This repository contains the **ESP32 transmitter and receiver firmware** for an ongoing modular UAV-control project.

The current system accepts structured commands from an external mobile application, sends them between two ESP32 nodes over **ESP-NOW**, applies receiver-side state and safety logic, and outputs **CRSF RC-channel frames** to a Betaflight flight controller. The receiver also parses CRSF return telemetry, including GPS data when available.

The project originally formed the embedded-control part of a bachelor thesis, but **development is not limited to the thesis scope**. The thesis evaluates a specific tested snapshot of the system; this repository is intended to continue evolving afterward.

### Current priorities

- low-latency command and feedback exchange;
- clear separation between app, transmitter, receiver and flight controller;
- receiver-local safety behaviour;
- CRSF flight-controller integration;
- GPS-assisted bounded movement where usable telemetry exists;
- useful diagnostics for continued bench and flight development.

---

## Architecture

```text
[ Mobile voice/control app ]
          |
          | USB serial / external app link
          v
[ Transmitter ESP32-WROOM-32U ]
          |
          | ESP-NOW
          v
[ Receiver ESP32-WROOM-32U ]
          |
          | state + safety logic
          | CRSF RC output / return telemetry
          v
[ Betaflight flight controller ]
          |
          +----> motors / ESC
          |
          +<---- GPS / FC telemetry
```

The phone is intentionally kept outside the time-critical receiver logic. Once a command has been converted into a supported token, the receiver owns the operational state, failsafe timers, emergency KILL behaviour and CRSF output.

---

## Repository structure

```text
DronerReciever/
  platformio.ini
  src/main.cpp
  ...

transmitterModule/
  platformio.ini
  src/main.cpp
  ...

README.md
```

Both firmware projects use **PlatformIO**, the Arduino framework and the `esp32dev` target.

---

## Transmitter

`transmitterModule` receives complete command lines from the external application, parses the command and optional distance value, assigns a sequence number and sends a packed `ControlPacket` through ESP-NOW.

It also receives `FeedbackPacket` messages from the receiver and records matching send/receive timing for command acknowledgement measurements.

### Supported command types

```text
STOP
ARM
DISARM
TAKEOFF
LAND
FORWARD
BACK
LEFT
RIGHT
YAW_LEFT
YAW_RIGHT
UP
DOWN
HOVER
KILL
PING
```

`PING` is used for startup and sequence resynchronization rather than normal flight control.

---

## ESP-NOW packet protocol

The current command packet is deliberately small:

```cpp
struct ControlPacket {
    uint32_t seq;
    uint8_t command;
    uint16_t distanceCm;
} __attribute__((packed));
```

The receiver returns richer execution feedback:

```cpp
struct FeedbackPacket {
    uint32_t seq;
    uint8_t command;
    uint8_t status;
    uint8_t state;
    uint16_t distanceCm;
    uint16_t progressCm;
    uint16_t throttleUs;
    uint16_t targetThrottleUs;
    uint16_t throttleCrsf;
    uint32_t receiverMillis;
} __attribute__((packed));
```

Feedback states include accepted commands, stale-sequence rejection, GPS-related fallback/rejection, invalid packets and commands rejected while KILL is latched.

The current sender is **event-driven**. It sends completed commands plus startup/resynchronization PING packets; it does not currently transmit a continuous heartbeat.

---

## Receiver state machine

The receiver firmware uses six explicit states:

- `IDLE`
- `TAKEOFF`
- `ACTIVE`
- `HOVER_FAILSAFE`
- `LAND_FAILSAFE`
- `KILL`

Normal movement changes the receiver's locally stored control intention. The receiver then continuously emits CRSF channel data independently of how often the operator speaks.

### Current timing parameters

- receiver loop delay: **20 ms**;
- HOVER failsafe threshold: **7 s** of accepted-command inactivity under eligible armed-flight conditions;
- LAND failsafe threshold: **30 s**;
- takeoff phase: approximately **2 s**;
- landing descent phase before forced disarm: approximately **5 s**.

These are prototype tuning values, not universal UAV safety limits.

---

## Emergency KILL

`KILL` is the highest-priority receiver state.

When activated, the receiver:

- forces throttle to minimum;
- neutralizes roll, pitch and yaw;
- forces the AUX1 arming channel low;
- bypasses ordinary output smoothing;
- remains latched in KILL;
- rejects normal post-KILL control until the receiver is manually restarted.

This logic is enforced on the aircraft-side receiver rather than relying on the phone application to remain connected.

---

## CRSF / Betaflight integration

The receiver generates packed CRSF RC-channel frames for the flight controller and parses the return serial telemetry stream.

Current PlatformIO configuration:

```text
CRSF TX: GPIO25
CRSF RX: GPIO26
CRSF baud: 420000
```

The thesis-tested integration used an **OmnibusF4SD / STM32F405** flight controller with Betaflight build **2026.6.1**.

The flight controller remains responsible for low-level stabilization and motor control. The ESP32 receiver supplies RC-equivalent command channels and adds the project-specific command/state/safety layer above it.

---

## GPS telemetry and bounded movement

The receiver parses both classic and extended CRSF GPS telemetry where available.

Current behaviour includes:

- recent-telemetry checks;
- satellite and coordinate validity checks;
- optional extended GPS quality checks;
- classic-GPS support for horizontal distance estimation when appropriate;
- stricter quality requirements for altitude-related assistance;
- timed movement fallback when usable GPS is unavailable at command start;
- transition to `HOVER_FAILSAFE` if a GPS-measured manoeuvre loses required telemetry while active.

GPS-assisted logic is still an area of active development. The project should not be interpreted as a finished precision-navigation or autonomous-positioning system.

---

## Hardware used during the thesis prototype

The thesis prototype used:

- two ESP32-WROOM-32U boards;
- ESP-NOW between transmitter and receiver;
- external antenna on the transmitter and a smaller whip antenna on the aircraft-side receiver;
- OmnibusF4SD / STM32F405 flight controller;
- 4-in-1 45 A ESC;
- 7-inch multirotor platform;
- BN-220 and BN-880 GPS modules during development;
- CRSF serial connection from the receiver ESP32 to the flight controller.

Hardware is expected to change as the project continues.

---

## Mobile application

The mobile application is maintained separately from this repository.

Its role is to:

1. capture or otherwise obtain the operator command;
2. map recognized input to the supported command vocabulary;
3. send a structured command line to the transmitter;
4. display returned acknowledgements, receiver state and timing information.

The embedded firmware does not depend on speech recognition internally; commands can also be sent directly over the transmitter serial interface for bench testing.

---

## Thesis snapshot

The bachelor thesis evaluates a **specific frozen implementation**, not every future revision of this repository.

The embedded transmitter/receiver revision referenced by the thesis is:

```text
56bcd75266fb3c0d48201aadda84b97a9fc09547
```

That revision includes the integrated ESP-NOW command/feedback path, CRSF output and telemetry, GPS parsing/quality logic, timed fallback behaviour, staged inactivity failsafes and latched KILL handling.

Future commits may intentionally change tuning, hardware assumptions, protocol behaviour or flight-control features. **Results reported in the thesis should therefore be associated with the frozen commit above rather than automatically attributed to the latest `master` branch.**

---

## Ongoing development / roadmap

This project is intended to continue beyond the thesis. Possible next steps include:

- mechanically stabilizing and retuning the airframe for repeatable free flight;
- improving GPS and flight-controller estimator integration;
- delegating precision altitude/position hold to appropriate FC-native control modes;
- controlled ESP-NOW range, obstacle and packet-delivery testing;
- adding an explicit heartbeat/link-health mechanism separate from command inactivity;
- stronger peer/source authentication and validation;
- improved synchronized logging across the app, transmitter, receiver and flight controller;
- better telemetry visualization and tuning tools;
- additional sensors and autonomous-assistance experiments;
- continued refinement of landing behaviour beyond fixed-time descent and forced disarm.

The roadmap is intentionally broader than the claims made in the thesis.

---

## Getting started

### Requirements

- two compatible ESP32 boards;
- PlatformIO;
- a serial terminal or compatible external control application;
- for FC integration, a Betaflight-compatible controller configured for CRSF Serial RX.

### Basic workflow

1. Clone the repository.
2. Open `transmitterModule` in PlatformIO and flash the transmitter ESP32.
3. Open `DronerReciever` in PlatformIO and flash the receiver ESP32.
4. Verify the configured ESP-NOW peer MAC address.
5. For flight-controller use, verify the CRSF UART wiring and Betaflight Serial RX configuration.
6. Perform propeller-free receiver/channel tests before attempting motor or flight testing.

---

## Safety notice

This software controls physical UAV hardware and is experimental. Incorrect wiring, configuration, tuning, command handling or failsafe behaviour can cause property damage or injury.

Use controlled test conditions, verify commands and failsafes without propellers first, and comply with applicable UAV regulations and local operating requirements.

---

## Author

**Mustafa Qassmieh**

Ongoing embedded/UAV project originally developed as part of a Computer Science bachelor thesis.
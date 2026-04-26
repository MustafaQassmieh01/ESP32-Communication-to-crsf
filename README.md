# 📡 Voice-Controlled UAV System (ESP32 + ESP-NOW)

## 📖 Overview

This repository contains the **transmitter and receiver implementation** for a modular, voice-controlled UAV system. The system enables real-time drone control using voice commands processed through a mobile device and transmitted to an ESP32-based control unit.

The architecture is designed with a strong focus on:

* **Low-latency communication**
* **Safety-critical control**
* **Modular system design**
* **Thesis-oriented evaluation (latency, reliability, accuracy)**

The implementation uses **ESP-NOW** for efficient peer-to-peer wireless communication between the transmitter and receiver.

---

## 🧠 System Architecture

The system follows a layered architecture:

```
[ Voice Input (Mobile App) ]
        ↓
[ Transmitter (ESP32U) ]
        ↓
[ ESP-NOW Communication ]
        ↓
[ Receiver (ESP32) ]
        ↓
[ Safety & Control Logic ]
        ↓
[ Flight Controller Output (CRSF planned) ]
```

This separation ensures modularity and easier testing of individual components. 

---

## 📂 Repository Structure

```
/transmitter
  ├── main.cpp
  ├── config.h
  └── ...

/receiver
  ├── main.cpp
  ├── config.h
  └── ...

/shared
  ├── packet_definitions.h
  └── ...
```

---

## 📡 Communication Protocol

Communication between transmitter and receiver is handled via ESP-NOW.

### Packet Structure (Example)

```cpp
struct ControlPacket {
  uint32_t seq;
  uint8_t command;
  int16_t throttle;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
};
```

### Key Features

* Sequence numbers prevent stale packet usage
* Receiver always applies **latest valid packet**
* Designed for **20–50 Hz update rate**
* Timeout-based failsafe handling

---

## 🎙 Command System

The system uses a **persistent command model**:

* Commands modify the current control state
* Movement continues until a new command is received

### Example Commands

```
FORWARD
BACK
LEFT
RIGHT
YAW_LEFT
YAW_RIGHT
STOP
TAKEOFF
LAND
KILL
```

### Behavior Rules

* `FORWARD` → continuous forward motion
* `LEFT/RIGHT` → directional adjustment
* `STOP` → return to neutral hover state
* No input → triggers failsafe

This aligns with a behavior-based control model. 

---

## 🧩 System States

The receiver operates using a state machine:

* `IDLE`
* `TAKEOFF`
* `ACTIVE`
* `HOVER_FAILSAFE`
* `LAND_FAILSAFE`
* `KILL`

Each state defines how control signals are generated and applied.

---

## 🧯 Safety System

Safety is a core design component of the system.

### Failsafe Logic

* **No signal for 7 seconds → HOVER_FAILSAFE**
* **No signal for 30 seconds → LAND_FAILSAFE**

Failsafe ensures controlled behavior under communication loss.

---

## 🛑 Kill Switch (Emergency Override)

The system implements a **latched kill switch**:

* Triggered from the **transmitter**
* Enforced on the **receiver**
* Overrides all commands and states

### Behavior

* Throttle → minimum immediately
* Roll / Pitch / Yaw → neutral
* Ignores all incoming commands
* Requires **manual receiver reset** to exit

This follows safety-critical system design principles. 

---

## ⚙️ Control Logic

### Default Control Values

The transmitter maintains safe defaults:

```cpp
hoverThrottle = 1450;
takeoffThrottle = 1600;
pitchStep = 100;
rollStep = 100;
yawStep = 100;
```

If no value is provided from the mobile app, defaults are used.

---

## 📱 Mobile App Integration (External)

The system supports input from a mobile app via:

* USB-C serial or Bluetooth
* Voice command interpretation

### Example Input

```
CMD:FORWARD
CMD:LEFT;YAW:1400
CMD:TAKEOFF;THR:1650
CMD:KILL
```

The transmitter applies defaults when parameters are missing. 

---

## 🧪 Testing & Evaluation (Thesis Focus)

The system is designed to support quantitative evaluation:

* **Latency (voice → execution)**
* **Command success rate**
* **Failsafe reliability**
* **Control stability**

Repeated trials are used to ensure consistency and reproducibility.

---

## 🚧 Planned Features

* CRSF protocol output to flight controller
* Adaptive hover throttle (EMA-based)
* Real-time telemetry feedback
* Live parameter tuning from mobile app
* Advanced smoothing and control filtering

---

## 🛠 Technologies Used

* ESP32 (ESP-NOW)
* C++ (Arduino / PlatformIO)
* Mobile voice input (external app)
* Serial / Bluetooth communication

---

## 📚 Academic Context

This project is part of a thesis focused on:

* Real-time embedded systems
* Voice-controlled human-drone interaction
* Safety-critical control design
* Modular system architecture

The design follows principles of:

* State-based control systems
* Behavior-based robotics
* Fault-tolerant communication

---

## 🚀 Getting Started

### Requirements

* ESP32 boards (transmitter + receiver)
* PlatformIO or Arduino IDE
* Mobile device (for voice input)

### Steps

1. Clone the repository
2. Flash transmitter code to ESP32U
3. Flash receiver code to ESP32
4. Configure MAC addresses for ESP-NOW
5. Connect mobile app to transmitter
6. Start sending commands

---

## ⚠️ Disclaimer

This project involves real-time control of physical hardware. Improper configuration may lead to unsafe behavior. Always test in a controlled environment.

---

## 👤 Author

Mustafa Qasmieh
Software Engineering Thesis Project

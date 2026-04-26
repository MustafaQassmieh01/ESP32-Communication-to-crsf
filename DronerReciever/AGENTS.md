# Project: Voice-Controlled Drone Receiver

## Goal
This project is part of a thesis about a modular voice-controlled drone system.

The receiver ESP32:
- receives control packets over ESP-NOW
- validates and tracks packet freshness
- converts high-level commands into RC channel targets
- applies smoothing and failsafe behavior
- outputs RC channel values through CRSF to a flight controller

## Architecture requirements
Keep the code modular. Separate logic into these responsibilities:

1. Transport layer
- ESP-NOW setup
- receive callback
- packet copy into a small shared buffer
- no heavy logic inside callback

2. Packet layer
- packet struct definitions
- validation
- sequence handling
- timeout tracking
- stale/replayed packet rejection if possible

3. Command/state layer
- state machine for drone control
- states should include:
  - IDLE
  - TAKEOFF
  - ACTIVE
  - HOVER_FAILSAFE
  - LAND_FAILSAFE

4. Control shaping layer
- map commands into target RC channel values
- allow tunable globals for testing
- support quick adjustment of:
  - hoverThrottle
  - powerStepPercent
  - angleStepPercent
  - takeoffLiftMultiplier
  - landingDropMultiplier
  - yawMultiplier
  - pitchMultiplier
  - rollMultiplier
  - throttleSlewPerTick
  - axisSlewPerTick

5. Output layer
- CRSF frame generation / sending
- isolate CRSF-specific logic from behavior logic

## Design preferences
- Prefer readable, testable functions over one huge file
- Avoid magic numbers
- Use clear names
- Add comments only where useful
- Do not remove working behavior unless replacing it with a cleaner equivalent
- Keep Arduino/ESP32 compatibility

## Safety / behavior rules
- Callback should do minimal work only
- Main loop handles packet processing
- Add timeout handling:
  - no valid packets for 7 seconds -> hover failsafe
  - no valid packets for 30 seconds -> landing failsafe
- Use smoothed transitions instead of sudden jumps
- Target channels and current channels should be separated

## Current control model
I want:
- helper functions to convert percentages into RC channel movement
- functions for commands like:
  - stop
  - hover
  - forward
  - back
  - left
  - right
  - yaw left
  - yaw right
  - up
  - down
- takeoff and landing should be state-driven, not just one-shot direct assignments

## Important implementation note
If something is unclear, preserve extensibility.
Prefer code skeletons and clean module boundaries over over-engineered complexity.
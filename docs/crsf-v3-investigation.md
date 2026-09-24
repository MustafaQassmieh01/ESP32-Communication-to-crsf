# CRSFv3 Investigation

This document tracks the first compatibility pass between the current ESP32 receiver implementation and the official Team BlackSheep CRSFv3 specification.

Related issues:

- #1 — Audit receiver implementation against CRSFv3 specification
- #2 — Prototype CRSFv3 baud-rate negotiation with safe 420k fallback
- #3 — Expand CRSF telemetry parser beyond GPS frames

## Current project baseline

The receiver currently:

- uses a dual-wire ESP32 UART connection to the Betaflight flight controller;
- starts the UART at 420000 baud;
- sends RC channels with broadcast frame type `0x16` (RC Channels Packed);
- packs 16 channels as 11-bit values into the standard 22-byte payload;
- uses CRC8 DVB-S2 (`0xD5`);
- accepts CRSF telemetry and currently decodes GPS `0x02` and GPS Extended `0x06`;
- limits CRSF frames to 64 bytes.

No flight-facing behavior is changed by this investigation commit.

## Initial CRSFv3 compatibility notes

### 1. RC channel framing is already on the normal CRSF path

The current `0x16` packed-channel output is a CRSF frame type documented by current flight-controller implementations. CRSFv3 work does not require replacing the project's RC control model just to begin compatibility work.

### 2. Baud-rate behavior needs an explicit compatibility strategy

The official TBS CRSFv3 specification states that dual-wire full-duplex UART defaults to **416666 baud** and may negotiate a higher baud rate.

The current project uses **420000 baud**. This is intentional for the existing Betaflight-facing prototype and must not be changed blindly. Betaflight and ExpressLRS commonly use 420000 in normal CRSF configurations.

Plan:

1. preserve 420000 as the known-good startup/fallback mode;
2. make the configured startup baud explicit;
3. implement negotiation behind a feature flag;
4. switch speeds only after a valid proposal/response exchange;
5. fall back safely if negotiation fails or times out.

### 3. CRSFv3 defines protocol-speed negotiation commands

The CRSFv3 spec defines speed negotiation under Direct Command frame type `0x32`:

- General command group `0x0A`
- `0x70` — CRSF Protocol Speed Proposal
- `0x71` — CRSF Protocol Speed Proposal Response

Supporting this requires an extended-header frame builder/parser rather than only broadcast frames.

### 4. The telemetry parser should tolerate compatible frame growth

CRSFv3 explicitly allows a frame to contain newer optional trailing fields. A receiver should not reject an otherwise known frame solely because its payload is longer than the minimum version it understands.

The current GPS handlers require an exact payload size:

- classic GPS: exactly 15 bytes;
- GPS Extended: exactly 20 bytes.

That is safe for the frames currently observed, but a CRSFv3-oriented parser should move toward **minimum known payload length** checks and ignore unknown trailing bytes.

### 5. Parser resynchronization deserves its own test

The current receive parser treats the first byte after reset as the start of a frame and validates the length byte next. It does not yet explicitly identify valid CRSF sync/address bytes before beginning a frame.

Before tightening that behavior, the implementation needs to account for the CRSFv3 rule that the first byte may be the serial sync byte, broadcast address, or a device address. A stricter parser should improve recovery from noise without incorrectly rejecting valid routed traffic.

### 6. Telemetry can become useful link-health data

The current receive side already has frame/CRC/error counters. CRSFv3 support can extend this with:

- Heartbeat `0x0B`;
- Link Statistics `0x14`;
- RX Link Statistics `0x1C`;
- TX Link Statistics `0x1D`;
- Battery Sensor `0x08`;
- Barometric Altitude / Vertical Speed `0x09`.

These should remain diagnostic/telemetry inputs first. They should not automatically modify flight behavior until their semantics and update rates are verified on the bench.

## First implementation sequence

1. Build a small CRSF protocol module instead of growing `main.cpp`.
2. Move frame constants, CRC, frame construction and frame parsing into that module.
3. Add tests or deterministic serial-frame fixtures for known frames.
4. Make telemetry handlers accept known minimum lengths.
5. Add extended-header support.
6. Add optional speed proposal/response handling.
7. Verify the unchanged 420000 path against Betaflight.
8. Bench-test any negotiated-speed mode before enabling it by default.

## References

- Team BlackSheep CRSFv3 specification: https://github.com/tbs-fpv/tbs-crsf-spec
- CRSFv3 protocol document: https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md
- Betaflight CRSF protocol definitions: https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf_protocol.h
- ExpressLRS receiver serial protocol documentation: https://www.expresslrs.org/software/serial-protocols/

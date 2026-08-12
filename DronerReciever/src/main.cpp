// ============================================================
// MARK: INCLUDES
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#ifndef CRSF_TX_PIN
#define CRSF_TX_PIN 25
#endif

#ifndef CRSF_RX_PIN
#define CRSF_RX_PIN 26
#endif

#ifndef CRSF_BAUD
#define CRSF_BAUD 420000
#endif

// ============================================================
// MARK: CONFIGURATION CONSTANTS
// ============================================================

namespace ReceiverConfig {
constexpr uint8_t kEspNowChannel = 1;
constexpr int8_t kCrsfTxPin = CRSF_TX_PIN;
constexpr int8_t kCrsfRxPin = CRSF_RX_PIN;
constexpr uint32_t kCrsfBaud = CRSF_BAUD;

constexpr uint16_t kRcMin = 1000;
constexpr uint16_t kRcMid = 1500;
constexpr uint16_t kRcMax = 2000;

constexpr uint8_t kDefaultPowerStepPercent = 12;
constexpr uint8_t kDefaultAngleStepPercent = 18;

constexpr uint32_t kHoverFailsafeMs = 7000; // Milliseconds without a command before hover failsafe.
constexpr uint32_t kLandFailsafeMs = 30000; // Milliseconds without a command before landing failsafe.
constexpr uint32_t kTakeoffDurationMs = 2000;
constexpr uint32_t kLandingDurationMs = 5000;
constexpr uint32_t kLoopDelayMs = 20;
constexpr uint32_t kDebugPrintIntervalMs = 500;
constexpr uint8_t kDistanceProfileMinPercent = 25;
constexpr uint8_t kMinGpsSatellites = 4;
constexpr uint32_t kGpsTelemetryTimeoutMs = 3000;
constexpr uint8_t kMinGpsFixTypeForMovement = 3;
constexpr uint16_t kMaxGpsHorizontalAccuracyCm = 250;
constexpr uint16_t kMaxGpsVerticalAccuracyCm = 500;
constexpr uint8_t kMaxGpsHDop10 = 30;
constexpr uint8_t kMaxGpsVDop10 = 50;
constexpr bool kAllowClassicGpsForHorizontalMovement = true;
constexpr bool kRequireExtendedGpsForAltitudeControl = true;
constexpr double kEarthRadiusCm = 637100000.0;

// Optional hover assist that uses GPS altitude to bias throttle around hoverThrottle.
constexpr bool kEnableGpsAltitudeHoldAssist = true;
// Ignore small altitude noise before applying any throttle correction.
constexpr int16_t kAltitudeHoldDeadbandCm = 120;
// Limit correction authority so GPS hold cannot command aggressive throttle steps.
constexpr uint8_t kAltitudeHoldMaxCorrectionPercent = 10;
// Proportional gain: throttle correction percentage per meter of altitude error.
constexpr float kAltitudeHoldKpPercentPerMeter = 4.0f;
}

namespace CrsfConfig {
constexpr uint8_t kAddressFlightController = 0xC8;
constexpr uint8_t kFrameTypeGps = 0x02;
constexpr uint8_t kFrameTypeGpsExtended = 0x06;
constexpr uint8_t kFrameTypeRcChannelsPacked = 0x16;
constexpr uint8_t kGpsPayloadSize = 15;
constexpr uint8_t kGpsExtendedPayloadSize = 20;
constexpr uint8_t kMaxFrameSize = 64;
constexpr uint8_t kRcPayloadSize = 22;
constexpr uint8_t kFrameSize = 26;
constexpr uint8_t kFrameLength = 24;
constexpr uint16_t kCrsfMin = 172;
constexpr uint16_t kCrsfMid = 992;
constexpr uint16_t kCrsfMax = 1811;
}

// ============================================================
// MARK: TUNING PARAMETERS
// ============================================================

struct VehicleTuning {
  // Throttle value used to hold a stable hover during testing.
  uint16_t hoverThrottle = 1400;

  // Multiplier applied during takeoff to briefly lift above hover thrust.
  float takeoffLiftMultiplier = 1.20f;

  // Multiplier applied during descent and landing behavior.
  float landingDropMultiplier = 0.60f;
};

struct CommandTuning {
  // Base throttle step in percent for up/down style commands.
  uint8_t powerStepPercent = ReceiverConfig::kDefaultPowerStepPercent;

  // Base axis deflection in percent for directional commands.
  uint8_t angleStepPercent = ReceiverConfig::kDefaultAngleStepPercent;

  // Scale factor for yaw responses.
  float yawMultiplier = 1.00f;

  // Scale factor for pitch responses.
  float pitchMultiplier = 1.00f;

  // Scale factor for roll responses.
  float rollMultiplier = 1.00f;
};

struct SafetyTuning {
  // Time without valid packets before entering hover failsafe.
  uint32_t hoverFailsafeMs = ReceiverConfig::kHoverFailsafeMs;

  // Time without valid packets before entering landing failsafe.
  uint32_t landFailsafeMs = ReceiverConfig::kLandFailsafeMs;

  // Time to hold boosted takeoff throttle before settling into hover.
  uint32_t takeoffDurationMs = ReceiverConfig::kTakeoffDurationMs;

  // Time to hold descent throttle before cutting throttle and disarming.
  uint32_t landingDurationMs = ReceiverConfig::kLandingDurationMs;

};

struct OutputSmoothingTuning {
  // Maximum throttle change applied per control tick.
  uint16_t throttleSlewPerTick = 10;

  // Maximum roll/pitch/yaw change applied per control tick.
  uint16_t axisSlewPerTick = 15;
};

// ============================================================
// MARK: ENUMS AND PACKET TYPES
// ============================================================

enum CommandType : uint8_t {
  CMD_STOP = 0,
  CMD_ARM,
  CMD_DISARM,
  CMD_TAKEOFF,
  CMD_LAND,
  CMD_FORWARD,
  CMD_BACK,
  CMD_LEFT,
  CMD_RIGHT,
  CMD_YAW_LEFT,
  CMD_YAW_RIGHT,
  CMD_UP,
  CMD_DOWN,
  CMD_HOVER,
  CMD_KILL, // failsafe command to immediately cut throttle, bypassing smoothing and safety checks. Should be used in emergencies only.
  CMD_PING
};

enum class DroneState : uint8_t {
  IDLE = 0,
  TAKEOFF,
  ACTIVE,
  HOVER_FAILSAFE,
  LAND_FAILSAFE,
  KILL
};

struct ControlPacket {
  uint32_t seq;
  uint8_t command;
  uint16_t distanceCm;
} __attribute__((packed));

enum FeedbackStatus : uint8_t {
  FEEDBACK_ACCEPTED = 0,
  FEEDBACK_REJECTED_STALE,
  FEEDBACK_REJECTED_GPS,
  FEEDBACK_GPS_FALLBACK,
  FEEDBACK_INVALID,
  FEEDBACK_REJECTED_KILL
};

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

struct RcChannels {
  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;
  uint16_t throttle;
  uint16_t aux1;
  uint16_t aux2;
  uint16_t aux3;
  uint16_t aux4;
};

struct LinkStats {
  bool hasSeenPacket = false;
  uint32_t lastAcceptedSeq = 0;
  uint32_t lastPacketAtMs = 0;
  uint32_t lastHoverFailsafeAtMs = 0;
  uint32_t lastLandingFailsafeAtMs = 0;
};

struct MotionTargets {
  int8_t pitchPercent = 0;
  int8_t rollPercent = 0;
  int8_t yawPercent = 0;
  int8_t throttlePercent = 0;
  bool useHoverThrottle = true;
  bool useMinimumThrottle = false;
};

struct DistanceMoveState {
  bool active = false;
  bool hasStartGps = false;
  bool useVerticalDistance = false;
  bool useTimeFallback = false;
  uint8_t command = CMD_HOVER;
  uint16_t targetDistanceCm = 0;
  uint16_t traveledDistanceCm = 0;
  uint32_t startedAtMs = 0;
  uint32_t targetDurationMs = 0;
  int32_t startLatitudeE7 = 0;
  int32_t startLongitudeE7 = 0;
  int32_t startAltitudeCm = 0;
};

struct GpsTelemetry {
  bool telemetryReceived = false;
  bool classicFrameReceived = false;
  bool extendedFrameReceived = false;
  int32_t latitudeE7 = 0;
  int32_t longitudeE7 = 0;
  int32_t altitudeCm = 0;
  uint16_t rawAltitude = 0;
  uint16_t groundSpeedKmh10 = 0;
  uint16_t headingDeg100 = 0;
  uint8_t satellites = 0;
  uint8_t fixType = 0;
  int16_t northSpeedCms = 0;
  int16_t eastSpeedCms = 0;
  int16_t upSpeedCms = 0;
  uint16_t horizontalSpeedAccuracyCms = 0;
  uint16_t trackAccuracyDeg10 = 0;
  int16_t ellipsoidAltitudeM = 0;
  uint16_t horizontalAccuracyCm = 0;
  uint16_t verticalAccuracyCm = 0;
  uint8_t hDop10 = 0;
  uint8_t vDop10 = 0;
  uint32_t lastTelemetryMs = 0;
  uint32_t lastClassicMs = 0;
  uint32_t lastExtendedMs = 0;
};

struct AltitudeHoldState {
  bool engaged = false;
  int32_t targetAltitudeCm = 0;
};

struct CrsfRxParser {
  uint8_t frame[CrsfConfig::kMaxFrameSize] = {};
  uint8_t index = 0;
  uint8_t expectedSize = 0;
};

struct CrsfTelemetryStats {
  uint32_t bytes = 0;
  uint32_t frames = 0;
  uint32_t gpsFrames = 0;
  uint32_t gpsExtendedFrames = 0;
  uint32_t crcErrors = 0;
  uint32_t sizeErrors = 0;
  uint32_t lastByteAtMs = 0;
  uint32_t lastFrameAtMs = 0;
  uint32_t lastGpsAtMs = 0;
};

// ============================================================
// MARK: GLOBAL STATE
// ============================================================

volatile bool gHasPendingPacket = false;
volatile bool gSawInvalidPacketSize = false;
volatile bool gKillLatched = false; // Latched emergency stop. Cleared only by receiver reset.
volatile bool gKillPendingProcessing = false;
volatile int gLastInvalidPacketSize = 0;
ControlPacket gPendingPacket = {};
uint8_t gPendingSenderMac[6] = {};
portMUX_TYPE gPacketMux = portMUX_INITIALIZER_UNLOCKED;

LinkStats gLinkStats;
DroneState gDroneState = DroneState::IDLE;
uint32_t gStateEnteredAtMs = 0;

// default neutral values for all channels, updated by command handlers and applied to currentChannels with smoothing in the main loop
RcChannels targetChannels = {
  ReceiverConfig::kRcMid,
  ReceiverConfig::kRcMid,
  ReceiverConfig::kRcMid,
  ReceiverConfig::kRcMin,
  ReceiverConfig::kRcMin,
  ReceiverConfig::kRcMin,
  ReceiverConfig::kRcMin,
  ReceiverConfig::kRcMin,
};

RcChannels currentChannels = targetChannels;

VehicleTuning gVehicleTuning;
CommandTuning gCommandTuning;
SafetyTuning gSafetyTuning;
OutputSmoothingTuning gOutputSmoothing;
DistanceMoveState gDistanceMove;
GpsTelemetry gGpsTelemetry;
CrsfRxParser gCrsfRxParser;
CrsfTelemetryStats gCrsfTelemetryStats;
AltitudeHoldState gAltitudeHold;

uint32_t gLastDebugPrintAtMs = 0;

void onReceive(const uint8_t *mac, const uint8_t *data, int len);

// ============================================================
// MARK: RC / CONTROL HELPER FUNCTIONS
// ============================================================

// Clamps a raw RC value to the valid range defined in ReceiverConfig.
uint16_t clampRc(int value) {
  if (value < ReceiverConfig::kRcMin) {
    return ReceiverConfig::kRcMin;
  }
  if (value > ReceiverConfig::kRcMax) {
    return ReceiverConfig::kRcMax;
  }
  return static_cast<uint16_t>(value);
}

// Converts a percentage input (-100 to 100) to a corresponding RC axis value, applying an optional multiplier for tuning.
uint16_t percentToAxis(int percent, float multiplier = 1.0f) {
  const int limitedPercent = constrain(percent, -100, 100);
  const float scaledPercent = static_cast<float>(limitedPercent) * multiplier;
  const int offset = static_cast<int>((500.0f * scaledPercent) / 100.0f);
  return clampRc(ReceiverConfig::kRcMid + offset);
}

// Converts a percentage input (0 to 100) to a corresponding RC throttle value, applying an optional multiplier for tuning.
uint16_t percentToThrottle(int percent, float multiplier = 1.0f) {
  const int limitedPercent = constrain(percent, 0, 100);
  const float scaledPercent = static_cast<float>(limitedPercent) * multiplier;
  const int offset = static_cast<int>((1000.0f * scaledPercent) / 100.0f);
  return clampRc(ReceiverConfig::kRcMin + offset);
}

uint8_t throttleToPercent(uint16_t throttleUs) {
  const uint16_t limited = clampRc(throttleUs);
  return static_cast<uint8_t>(((static_cast<uint32_t>(limited - ReceiverConfig::kRcMin)) * 100U) / 1000U);
}

// Smoothly slews a current RC value towards a target value by a specified step amount, ensuring it does not overshoot the target.
uint16_t slewTowards(uint16_t current, uint16_t target, uint16_t step) {
  if (current == target || step == 0) {
    return target;
  }
  if (current < target) {
    const uint32_t next = static_cast<uint32_t>(current) + step;
    return static_cast<uint16_t>(next > target ? target : next);
  }
  const int next = static_cast<int>(current) - static_cast<int>(step);
  return static_cast<uint16_t>(next < target ? target : next);
}

uint8_t crc8DvbS2(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}


int32_t readBigEndianInt32(const uint8_t *data) {
  return static_cast<int32_t>(
      (static_cast<uint32_t>(data[0]) << 24) |
      (static_cast<uint32_t>(data[1]) << 16) |
      (static_cast<uint32_t>(data[2]) << 8) |
      static_cast<uint32_t>(data[3]));
}

uint16_t readBigEndianUint16(const uint8_t *data) {
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

int16_t readBigEndianInt16(const uint8_t *data) {
  return static_cast<int16_t>(readBigEndianUint16(data));
}

uint16_t clampDistanceCm(uint32_t distanceCm) {
  return distanceCm > 65535U ? 65535U : static_cast<uint16_t>(distanceCm);
}

bool isVerticalDistanceCommand(uint8_t command);

bool hasRecentGpsTelemetry() {
  return gGpsTelemetry.telemetryReceived &&
         millis() - gGpsTelemetry.lastTelemetryMs <= ReceiverConfig::kGpsTelemetryTimeoutMs;
}

bool hasRecentGpsExtendedTelemetry() {
  return gGpsTelemetry.extendedFrameReceived &&
         millis() - gGpsTelemetry.lastExtendedMs <= ReceiverConfig::kGpsTelemetryTimeoutMs;
}

bool hasValidClassicGpsPosition() {
  if (!hasRecentGpsTelemetry() || !gGpsTelemetry.classicFrameReceived) {
    return false;
  }
  if (gGpsTelemetry.satellites < ReceiverConfig::kMinGpsSatellites) {
    return false;
  }
  if (gGpsTelemetry.latitudeE7 == 0 && gGpsTelemetry.longitudeE7 == 0) {
    return false;
  }

  return gGpsTelemetry.latitudeE7 >= -900000000L &&
         gGpsTelemetry.latitudeE7 <= 900000000L &&
         gGpsTelemetry.longitudeE7 >= -1800000000L &&
         gGpsTelemetry.longitudeE7 <= 1800000000L;
}

bool hasExtendedGpsQuality() {
  if (!hasRecentGpsExtendedTelemetry()) {
    return false;
  }

  return gGpsTelemetry.fixType >= ReceiverConfig::kMinGpsFixTypeForMovement &&
         gGpsTelemetry.horizontalAccuracyCm > 0 &&
         gGpsTelemetry.verticalAccuracyCm > 0 &&
         gGpsTelemetry.horizontalAccuracyCm <= ReceiverConfig::kMaxGpsHorizontalAccuracyCm &&
         gGpsTelemetry.verticalAccuracyCm <= ReceiverConfig::kMaxGpsVerticalAccuracyCm &&
         gGpsTelemetry.hDop10 > 0 &&
         gGpsTelemetry.vDop10 > 0 &&
         gGpsTelemetry.hDop10 <= ReceiverConfig::kMaxGpsHDop10 &&
         gGpsTelemetry.vDop10 <= ReceiverConfig::kMaxGpsVDop10;
}

bool hasValidGpsFix() {
  if (hasRecentGpsExtendedTelemetry()) {
    return hasValidClassicGpsPosition() &&
           gGpsTelemetry.fixType >= ReceiverConfig::kMinGpsFixTypeForMovement;
  }

  return hasValidClassicGpsPosition();
}

bool hasGpsQualityForHorizontalMovement() {
  if (hasRecentGpsExtendedTelemetry()) {
    return hasValidClassicGpsPosition() && hasExtendedGpsQuality();
  }

  return ReceiverConfig::kAllowClassicGpsForHorizontalMovement && hasValidClassicGpsPosition();
}

bool hasGpsQualityForAltitudeControl() {
  if (ReceiverConfig::kRequireExtendedGpsForAltitudeControl || hasRecentGpsExtendedTelemetry()) {
    return hasValidClassicGpsPosition() && hasExtendedGpsQuality();
  }

  return hasValidClassicGpsPosition();
}

bool hasGpsQualityForDistanceCommand(uint8_t command) {
  return isVerticalDistanceCommand(command)
      ? hasGpsQualityForAltitudeControl()
      : hasGpsQualityForHorizontalMovement();
}

const char* gpsQualitySourceForDebug() {
  if (hasRecentGpsExtendedTelemetry()) {
    return "EXTENDED";
  }
  if (ReceiverConfig::kAllowClassicGpsForHorizontalMovement && hasValidClassicGpsPosition()) {
    return "CLASSIC";
  }
  return "NONE";
}

bool isVerticalDistanceCommand(uint8_t command) {
  return command == CMD_UP || command == CMD_DOWN;
}

uint16_t horizontalDistanceCm(int32_t startLatitudeE7, int32_t startLongitudeE7, int32_t currentLatitudeE7, int32_t currentLongitudeE7) {
  const double degToRad = 0.017453292519943295;
  const double startLatitudeRad = (static_cast<double>(startLatitudeE7) / 10000000.0) * degToRad;
  const double currentLatitudeRad = (static_cast<double>(currentLatitudeE7) / 10000000.0) * degToRad;
  const double deltaLatitudeRad = currentLatitudeRad - startLatitudeRad;
  const double deltaLongitudeRad = ((static_cast<double>(currentLongitudeE7 - startLongitudeE7)) / 10000000.0) * degToRad;
  const double x = deltaLongitudeRad * cos((startLatitudeRad + currentLatitudeRad) * 0.5);
  const double y = deltaLatitudeRad;
  const double distanceCm = sqrt((x * x) + (y * y)) * ReceiverConfig::kEarthRadiusCm;

  if (distanceCm <= 0.0) {
    return 0;
  }
  return clampDistanceCm(static_cast<uint32_t>(distanceCm + 0.5));
}

void processGpsTelemetryPayload(const uint8_t *payload, uint8_t payloadLen) {
  if (payloadLen != CrsfConfig::kGpsPayloadSize) {
    return;
  }

  const uint32_t now = millis();
  gGpsTelemetry.latitudeE7 = readBigEndianInt32(&payload[0]);
  gGpsTelemetry.longitudeE7 = readBigEndianInt32(&payload[4]);
  gGpsTelemetry.groundSpeedKmh10 = readBigEndianUint16(&payload[8]);
  gGpsTelemetry.headingDeg100 = readBigEndianUint16(&payload[10]);
  gGpsTelemetry.rawAltitude = readBigEndianUint16(&payload[12]);
  gGpsTelemetry.altitudeCm = (static_cast<int32_t>(gGpsTelemetry.rawAltitude) - 1000) * 100;
  gGpsTelemetry.satellites = payload[14];
  gGpsTelemetry.telemetryReceived = true;
  gGpsTelemetry.classicFrameReceived = true;
  gGpsTelemetry.lastTelemetryMs = now;
  gGpsTelemetry.lastClassicMs = now;
  gCrsfTelemetryStats.gpsFrames++;
  gCrsfTelemetryStats.lastGpsAtMs = now;
}

void processGpsExtendedTelemetryPayload(const uint8_t *payload, uint8_t payloadLen) {
  if (payloadLen != CrsfConfig::kGpsExtendedPayloadSize) {
    return;
  }

  const uint32_t now = millis();
  gGpsTelemetry.fixType = payload[0];
  gGpsTelemetry.northSpeedCms = readBigEndianInt16(&payload[1]);
  gGpsTelemetry.eastSpeedCms = readBigEndianInt16(&payload[3]);
  gGpsTelemetry.upSpeedCms = readBigEndianInt16(&payload[5]);
  gGpsTelemetry.horizontalSpeedAccuracyCms = readBigEndianUint16(&payload[7]);
  gGpsTelemetry.trackAccuracyDeg10 = readBigEndianUint16(&payload[9]);
  gGpsTelemetry.ellipsoidAltitudeM = readBigEndianInt16(&payload[11]);
  gGpsTelemetry.horizontalAccuracyCm = readBigEndianUint16(&payload[13]);
  gGpsTelemetry.verticalAccuracyCm = readBigEndianUint16(&payload[15]);
  gGpsTelemetry.hDop10 = payload[18];
  gGpsTelemetry.vDop10 = payload[19];
  gGpsTelemetry.telemetryReceived = true;
  gGpsTelemetry.extendedFrameReceived = true;
  gGpsTelemetry.lastTelemetryMs = now;
  gGpsTelemetry.lastExtendedMs = now;
  gCrsfTelemetryStats.gpsExtendedFrames++;
  gCrsfTelemetryStats.lastGpsAtMs = now;
}

void processCrsfTelemetryFrame(const uint8_t *frame, uint8_t frameSize) {
  if (frameSize < 5) {
    return;
  }

  const uint8_t frameLength = frame[1];
  const uint8_t frameType = frame[2];
  const uint8_t expectedCrc = frame[frameSize - 1];
  const uint8_t actualCrc = crc8DvbS2(&frame[2], frameLength - 1);
  if (expectedCrc != actualCrc) {
    gCrsfTelemetryStats.crcErrors++;
    return;
  }

  gCrsfTelemetryStats.frames++;
  gCrsfTelemetryStats.lastFrameAtMs = millis();

  const uint8_t payloadLen = frameLength - 2;
  const uint8_t *payload = &frame[3];
  if (frameType == CrsfConfig::kFrameTypeGps) {
    processGpsTelemetryPayload(payload, payloadLen);
  } else if (frameType == CrsfConfig::kFrameTypeGpsExtended) {
    processGpsExtendedTelemetryPayload(payload, payloadLen);
  }
}

void resetCrsfRxParser() {
  gCrsfRxParser.index = 0;
  gCrsfRxParser.expectedSize = 0;
}

void feedCrsfTelemetryByte(uint8_t byte) {
  if (gCrsfRxParser.index == 0) {
    gCrsfRxParser.frame[gCrsfRxParser.index++] = byte;
    return;
  }

  if (gCrsfRxParser.index == 1) {
    if (byte < 2 || byte + 2 > CrsfConfig::kMaxFrameSize) {
      gCrsfTelemetryStats.sizeErrors++;
      resetCrsfRxParser();
      return;
    }

    gCrsfRxParser.frame[gCrsfRxParser.index++] = byte;
    gCrsfRxParser.expectedSize = byte + 2;
    return;
  }

  gCrsfRxParser.frame[gCrsfRxParser.index++] = byte;
  if (gCrsfRxParser.index >= gCrsfRxParser.expectedSize) {
    processCrsfTelemetryFrame(gCrsfRxParser.frame, gCrsfRxParser.expectedSize);
    resetCrsfRxParser();
  }
}

void readCrsfTelemetry() {
  if (ReceiverConfig::kCrsfRxPin < 0) {
    return;
  }

  while (Serial2.available() > 0) {
    gCrsfTelemetryStats.bytes++;
    gCrsfTelemetryStats.lastByteAtMs = millis();
    feedCrsfTelemetryByte(static_cast<uint8_t>(Serial2.read()));
  }
}
uint16_t rcUsToCrsf(uint16_t us) {
  const uint16_t limitedUs = clampRc(us);
  const int32_t centered = static_cast<int32_t>(limitedUs) - ReceiverConfig::kRcMid;
  const int32_t crsf = CrsfConfig::kCrsfMid + ((centered * 819) / 500);
  return constrain(crsf, CrsfConfig::kCrsfMin, CrsfConfig::kCrsfMax);
}

void setNeutralTargets() {
  targetChannels.roll = ReceiverConfig::kRcMid;
  targetChannels.pitch = ReceiverConfig::kRcMid;
  targetChannels.yaw = ReceiverConfig::kRcMid;
}

void setThrottleTarget(uint16_t throttle) {
  targetChannels.throttle = clampRc(throttle);
}

void resetAltitudeHold() {
  // Force next hover entry to capture a fresh reference altitude.
  gAltitudeHold = AltitudeHoldState{};
}

void captureAltitudeHoldTargetIfNeeded() {
  if (gAltitudeHold.engaged || !hasGpsQualityForAltitudeControl()) {
    return;
  }

  gAltitudeHold.targetAltitudeCm = gGpsTelemetry.altitudeCm;
  gAltitudeHold.engaged = true;
}

uint16_t applyAltitudeHoldAssist(uint16_t baseHoverThrottleUs) {
  // Fallback to static hover throttle when altitude-quality GPS is unavailable.
  if (!ReceiverConfig::kEnableGpsAltitudeHoldAssist || !hasGpsQualityForAltitudeControl()) {
    return baseHoverThrottleUs;
  }

  captureAltitudeHoldTargetIfNeeded();
  if (!gAltitudeHold.engaged) {
    return baseHoverThrottleUs;
  }

  const int32_t errorCm = gAltitudeHold.targetAltitudeCm - gGpsTelemetry.altitudeCm;
  // Deadband avoids hunting from GPS altitude jitter around the target.
  if (abs(errorCm) <= ReceiverConfig::kAltitudeHoldDeadbandCm) {
    return baseHoverThrottleUs;
  }

  const float errorMeters = static_cast<float>(errorCm) / 100.0f;
  int correctionPercent = static_cast<int>(errorMeters * ReceiverConfig::kAltitudeHoldKpPercentPerMeter);
  correctionPercent = constrain(
      correctionPercent,
      -static_cast<int>(ReceiverConfig::kAltitudeHoldMaxCorrectionPercent),
      static_cast<int>(ReceiverConfig::kAltitudeHoldMaxCorrectionPercent));

  const int hoverPercent = static_cast<int>(throttleToPercent(baseHoverThrottleUs));
  const int adjustedPercent = constrain(hoverPercent + correctionPercent, 0, 100);
  return percentToThrottle(adjustedPercent);
}

void setHoverThrottleTarget() {
  // Hover throttle remains the baseline; assist only applies bounded bias.
  const uint16_t baseHoverThrottle = clampRc(gVehicleTuning.hoverThrottle);
  setThrottleTarget(applyAltitudeHoldAssist(baseHoverThrottle));
}

void setArmTarget(bool armed) {
  targetChannels.aux1 = armed ? ReceiverConfig::kRcMax : ReceiverConfig::kRcMin;
}

uint16_t makeThrottleOffsetFromHover(int offsetPercent) {
  const int limitedOffset = constrain(offsetPercent, -100, 100);
  const int hover = static_cast<int>(clampRc(gVehicleTuning.hoverThrottle));
  const int offsetUs = (limitedOffset * 1000) / 100;
  return clampRc(hover + offsetUs);
}

uint16_t makeDescentThrottle(int percent, float multiplier) {
  return makeThrottleOffsetFromHover(-static_cast<int>(percent * multiplier));
}

uint16_t makePitchTarget(int percent) {
  return percentToAxis(percent, gCommandTuning.pitchMultiplier);
}

uint16_t makeRollTarget(int percent) {
  return percentToAxis(percent, gCommandTuning.rollMultiplier);
}

uint16_t makeYawTarget(int percent) {
  return percentToAxis(percent, gCommandTuning.yawMultiplier);
}

const char* stateToString(DroneState state) {
  switch (state) {
    case DroneState::IDLE: return "IDLE";
    case DroneState::TAKEOFF: return "TAKEOFF";
    case DroneState::ACTIVE: return "ACTIVE";
    case DroneState::HOVER_FAILSAFE: return "HOVER_FAILSAFE";
    case DroneState::LAND_FAILSAFE: return "LAND_FAILSAFE";
    case DroneState::KILL: return "KILL";
    default: return "UNKNOWN";
  }
}

void enterDroneState(DroneState state) {
  gDroneState = state;
  gStateEnteredAtMs = millis();
}

uint32_t elapsedInStateMs() {
  return millis() - gStateEnteredAtMs;
}

const char* commandToString(uint8_t command) {
  switch (command) {
    case CMD_STOP: return "STOP";
    case CMD_ARM: return "ARM";
    case CMD_DISARM: return "DISARM";
    case CMD_TAKEOFF: return "TAKEOFF";
    case CMD_LAND: return "LAND";
    case CMD_FORWARD: return "FORWARD";
    case CMD_BACK: return "BACK";
    case CMD_LEFT: return "LEFT";
    case CMD_RIGHT: return "RIGHT";
    case CMD_YAW_LEFT: return "YAW_LEFT";
    case CMD_YAW_RIGHT: return "YAW_RIGHT";
    case CMD_UP: return "UP";
    case CMD_DOWN: return "DOWN";
    case CMD_HOVER: return "HOVER";
    case CMD_KILL: return "KILL";
    case CMD_PING: return "PING";
    default: return "UNKNOWN_CMD";
  }
}
// ============================================================
// MARK: COMMAND BUILDERS
// ============================================================

MotionTargets cmdStop() {
  MotionTargets motion;
  motion.useHoverThrottle = false;
  motion.useMinimumThrottle = true;
  motion.throttlePercent = 0;
  return motion;
}

MotionTargets cmdHover() {
  return MotionTargets {};
}

MotionTargets cmdForward() {
  MotionTargets motion;
  motion.pitchPercent = gCommandTuning.angleStepPercent;
  return motion;
}

MotionTargets cmdBack() {
  MotionTargets motion;
  motion.pitchPercent = -static_cast<int8_t>(gCommandTuning.angleStepPercent);
  return motion;
}

MotionTargets cmdLeft() {
  MotionTargets motion;
  motion.rollPercent = -static_cast<int8_t>(gCommandTuning.angleStepPercent);
  return motion;
}

MotionTargets cmdRight() {
  MotionTargets motion;
  motion.rollPercent = gCommandTuning.angleStepPercent;
  return motion;
}

MotionTargets cmdYawLeft() {
  MotionTargets motion;
  motion.yawPercent = -static_cast<int8_t>(gCommandTuning.angleStepPercent);
  return motion;
}

MotionTargets cmdYawRight() {
  MotionTargets motion;
  motion.yawPercent = gCommandTuning.angleStepPercent;
  return motion;
}

MotionTargets cmdUp() {
  MotionTargets motion;
  motion.useHoverThrottle = false;
  motion.throttlePercent = gCommandTuning.powerStepPercent;
  return motion;
}

MotionTargets cmdDown() {
  MotionTargets motion;
  motion.useHoverThrottle = false;
  motion.throttlePercent = -static_cast<int8_t>(gCommandTuning.powerStepPercent * gVehicleTuning.landingDropMultiplier);
  return motion;
}

void applyMotionTargets(const MotionTargets &motion) {
  setNeutralTargets();
  targetChannels.pitch = makePitchTarget(motion.pitchPercent);
  targetChannels.roll = makeRollTarget(motion.rollPercent);
  targetChannels.yaw = makeYawTarget(motion.yawPercent);

  if (motion.useMinimumThrottle) {
    resetAltitudeHold();
    setThrottleTarget(ReceiverConfig::kRcMin);
  } else if (motion.useHoverThrottle) {
    setHoverThrottleTarget();
  } else {
    // Any non-hover motion should release altitude hold so the next hover locks to
    // the new local altitude instead of a stale previous target.
    resetAltitudeHold();
    setThrottleTarget(makeThrottleOffsetFromHover(motion.throttlePercent));
  }
}

// ============================================================

bool isDistanceCommand(uint8_t command) {
  return command == CMD_FORWARD ||
         command == CMD_BACK ||
         command == CMD_LEFT ||
         command == CMD_RIGHT ||
         command == CMD_UP ||
         command == CMD_DOWN;
}

void clearDistanceMove() {
  gDistanceMove = DistanceMoveState{};
}

void startDistanceMove(uint8_t command, uint16_t distanceCm, bool useTimeFallback) {
  gDistanceMove.active = distanceCm > 0;
  gDistanceMove.hasStartGps = !useTimeFallback && hasGpsQualityForDistanceCommand(command);
  gDistanceMove.useVerticalDistance = isVerticalDistanceCommand(command);
  gDistanceMove.useTimeFallback = useTimeFallback;
  gDistanceMove.command = command;
  gDistanceMove.targetDistanceCm = distanceCm;
  gDistanceMove.traveledDistanceCm = 0;
  gDistanceMove.startedAtMs = millis();
  gDistanceMove.targetDurationMs = static_cast<uint32_t>(distanceCm) * 10U;
  gDistanceMove.startLatitudeE7 = gGpsTelemetry.latitudeE7;
  gDistanceMove.startLongitudeE7 = gGpsTelemetry.longitudeE7;
  gDistanceMove.startAltitudeCm = gGpsTelemetry.altitudeCm;
}

uint16_t getDistanceMoveProgressCm() {
  if (gDistanceMove.useTimeFallback) {
    const uint32_t elapsedMs = millis() - gDistanceMove.startedAtMs;
    const uint32_t progressUnits = elapsedMs / 10U;
    return clampDistanceCm(progressUnits > gDistanceMove.targetDistanceCm ? gDistanceMove.targetDistanceCm : progressUnits);
  }

  if (!gDistanceMove.hasStartGps || !hasGpsQualityForDistanceCommand(gDistanceMove.command)) {
    return gDistanceMove.traveledDistanceCm;
  }

  if (gDistanceMove.useVerticalDistance) {
    const int32_t altitudeDeltaCm = gGpsTelemetry.altitudeCm - gDistanceMove.startAltitudeCm;
    const uint32_t absoluteDeltaCm = altitudeDeltaCm < 0 ? static_cast<uint32_t>(-altitudeDeltaCm) : static_cast<uint32_t>(altitudeDeltaCm);
    return clampDistanceCm(absoluteDeltaCm);
  }

  return horizontalDistanceCm(
      gDistanceMove.startLatitudeE7,
      gDistanceMove.startLongitudeE7,
      gGpsTelemetry.latitudeE7,
      gGpsTelemetry.longitudeE7);
}

uint8_t computeDistanceProfileScalePercent(uint16_t targetDistanceCm, uint16_t traveledDistanceCm) {
  if (targetDistanceCm == 0) {
    return 100;
  }
  if (traveledDistanceCm >= targetDistanceCm) {
    return 0;
  }

  uint16_t halfDistanceCm = (targetDistanceCm + 1) / 2;
  if (halfDistanceCm == 0) {
    halfDistanceCm = 1;
  }
  const uint16_t remainingDistanceCm = targetDistanceCm - traveledDistanceCm;
  const uint16_t rampDistanceCm = traveledDistanceCm < remainingDistanceCm ? traveledDistanceCm : remainingDistanceCm;
  const uint8_t scalePercent = static_cast<uint8_t>((static_cast<uint32_t>(rampDistanceCm) * 100U) / halfDistanceCm);

  return scalePercent < ReceiverConfig::kDistanceProfileMinPercent ? ReceiverConfig::kDistanceProfileMinPercent : scalePercent;
}

int8_t scaleMotionPercent(int8_t percent, uint8_t scalePercent) {
  return static_cast<int8_t>((static_cast<int16_t>(percent) * scalePercent) / 100);
}

MotionTargets applyDistanceProfile(const MotionTargets& motion, uint16_t targetDistanceCm, uint16_t traveledDistanceCm) {
  MotionTargets profiledMotion = motion;
  const uint8_t scalePercent = computeDistanceProfileScalePercent(targetDistanceCm, traveledDistanceCm);
  profiledMotion.pitchPercent = scaleMotionPercent(profiledMotion.pitchPercent, scalePercent);
  profiledMotion.rollPercent = scaleMotionPercent(profiledMotion.rollPercent, scalePercent);
  profiledMotion.yawPercent = scaleMotionPercent(profiledMotion.yawPercent, scalePercent);
  profiledMotion.throttlePercent = scaleMotionPercent(profiledMotion.throttlePercent, scalePercent);
  return profiledMotion;
}

MotionTargets motionForCommand(uint8_t command) {
  switch (command) {
    case CMD_FORWARD: return cmdForward();
    case CMD_BACK: return cmdBack();
    case CMD_LEFT: return cmdLeft();
    case CMD_RIGHT: return cmdRight();
    case CMD_YAW_LEFT: return cmdYawLeft();
    case CMD_YAW_RIGHT: return cmdYawRight();
    case CMD_UP: return cmdUp();
    case CMD_DOWN: return cmdDown();
    case CMD_DISARM: return cmdStop();
    case CMD_STOP: return cmdHover();
    case CMD_HOVER:
    default: return cmdHover();
  }
}
// MARK: STATE TARGET UPDATE LOGIC
// ============================================================

void updateStateTargets() {
  switch (gDroneState) {
    case DroneState::IDLE:
      applyMotionTargets(cmdStop());
      break;
    case DroneState::TAKEOFF: {
      if (elapsedInStateMs() >= gSafetyTuning.takeoffDurationMs) {
        enterDroneState(DroneState::ACTIVE);
        applyMotionTargets(cmdHover());
        Serial.println("Takeoff complete, entering hover");
        break;
      }

      MotionTargets motion = cmdHover();
      motion.useHoverThrottle = false;
      motion.throttlePercent = static_cast<int8_t>(
          gCommandTuning.powerStepPercent * gVehicleTuning.takeoffLiftMultiplier);
      applyMotionTargets(motion);
      break;
    }
    case DroneState::ACTIVE:
      if (gDistanceMove.active) {
        // Abort GPS-guided distance motion if GPS quality drops. Time fallback keeps running without GPS.
        if (!gDistanceMove.useTimeFallback && !hasGpsQualityForDistanceCommand(gDistanceMove.command)) {
          clearDistanceMove();
          enterDroneState(DroneState::HOVER_FAILSAFE);
          applyMotionTargets(cmdHover());
          Serial.println("Distance move aborted: GPS lost, entering hover failsafe");
          break;
        }

        gDistanceMove.traveledDistanceCm = getDistanceMoveProgressCm();
        if (gDistanceMove.traveledDistanceCm >= gDistanceMove.targetDistanceCm) {
          clearDistanceMove();
          applyMotionTargets(cmdHover());
        } else {
          MotionTargets motion = motionForCommand(gDistanceMove.command);
          motion = applyDistanceProfile(motion, gDistanceMove.targetDistanceCm, gDistanceMove.traveledDistanceCm);
          applyMotionTargets(motion);
        }
      }
      break;
    case DroneState::HOVER_FAILSAFE:
      applyMotionTargets(cmdHover());
      break;
    case DroneState::LAND_FAILSAFE: {
      if (elapsedInStateMs() >= gSafetyTuning.landingDurationMs) {
        enterDroneState(DroneState::IDLE);
        resetAltitudeHold();
        setArmTarget(false);
        applyMotionTargets(cmdStop());
        Serial.println("Landing complete, throttle cut and disarmed");
        break;
      }

      MotionTargets motion = cmdHover();
      motion.useHoverThrottle = false;
      motion.throttlePercent = -static_cast<int8_t>(
          gCommandTuning.powerStepPercent * gVehicleTuning.landingDropMultiplier);
      applyMotionTargets(motion);
      break;
    }
    case DroneState::KILL:
      applyMotionTargets(cmdStop());
      break;
  }
}

void applyCommand(const ControlPacket &packet) {
  MotionTargets motion;
  bool applyMotion = true;

  if (packet.command == CMD_KILL) {
    clearDistanceMove();
    gKillLatched = true;
    setArmTarget(false);
    enterDroneState(DroneState::KILL);
    motion = cmdStop();
    applyMotionTargets(motion);
    currentChannels = targetChannels;
    return;
  }

  if (gKillLatched) {
    return;
  }

  if (isDistanceCommand(packet.command) && packet.distanceCm > 0) {
    if (!hasGpsQualityForDistanceCommand(packet.command)) {
      enterDroneState(DroneState::ACTIVE);
      startDistanceMove(packet.command, packet.distanceCm, true);
      motion = motionForCommand(packet.command);
      motion = applyDistanceProfile(motion, gDistanceMove.targetDistanceCm, gDistanceMove.traveledDistanceCm);
      applyMotionTargets(motion);
      Serial.println("Distance command using no-GPS timed fallback");
      return;
    }

    enterDroneState(DroneState::ACTIVE);
    startDistanceMove(packet.command, packet.distanceCm, false);
    motion = motionForCommand(packet.command);
    motion = applyDistanceProfile(motion, gDistanceMove.targetDistanceCm, gDistanceMove.traveledDistanceCm);
    applyMotionTargets(motion);
    return;
  }


  clearDistanceMove();

  switch (packet.command) {
    case CMD_DISARM:
      enterDroneState(DroneState::IDLE);
      resetAltitudeHold();
      setArmTarget(false);
      motion = cmdStop();
      break;
    case CMD_ARM:
      setArmTarget(true);
      enterDroneState(DroneState::ACTIVE);
      motion = cmdStop();
      break;
    case CMD_STOP:
    case CMD_HOVER:
      enterDroneState(DroneState::ACTIVE);
      motion = cmdHover();
      break;
    case CMD_TAKEOFF:
      enterDroneState(DroneState::TAKEOFF);
      applyMotion = false;
      break;
    case CMD_LAND:
      enterDroneState(DroneState::LAND_FAILSAFE);
      applyMotion = false;
      break;
    default:
      enterDroneState(DroneState::ACTIVE);
      motion = motionForCommand(packet.command);
      break;
  }

  if (applyMotion) {
    applyMotionTargets(motion);
  } else {
    updateStateTargets();
  }
}

// ============================================================

bool ensureEspNowPeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = ReceiverConfig::kEspNowChannel;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK) {
    Serial.print("Failed to add feedback peer, error=");
    Serial.println(result);
    return false;
  }

  return true;
}

void sendFeedbackPacket(const uint8_t* senderMac, const ControlPacket& packet, uint8_t status) {
  if (!ensureEspNowPeer(senderMac)) {
    return;
  }

  FeedbackPacket feedback{};
  feedback.seq = packet.seq;
  feedback.command = packet.command;
  feedback.status = status;
  feedback.state = static_cast<uint8_t>(gDroneState);
  feedback.distanceCm = packet.distanceCm;
  feedback.progressCm = gDistanceMove.traveledDistanceCm;
  feedback.throttleUs = currentChannels.throttle;
  feedback.targetThrottleUs = targetChannels.throttle;
  feedback.throttleCrsf = rcUsToCrsf(currentChannels.throttle);
  feedback.receiverMillis = millis();

  esp_err_t result = esp_now_send(senderMac, reinterpret_cast<const uint8_t*>(&feedback), sizeof(feedback));
  if (result != ESP_OK) {
    Serial.print("Feedback send failed, error=");
    Serial.println(result);
  }
}
// MARK: PACKET HANDLING
// ============================================================

bool isSequenceNewer(uint32_t seq, uint32_t reference) {
  return static_cast<int32_t>(seq - reference) > 0;
}

bool fetchPendingPacket(ControlPacket &packetOut, uint8_t senderMacOut[6]) {
  bool hasPacket = false;

  portENTER_CRITICAL(&gPacketMux);
  if (gHasPendingPacket) {
    packetOut = gPendingPacket;
    memcpy(senderMacOut, gPendingSenderMac, 6);
    gHasPendingPacket = false;
    hasPacket = true;
  }
  portEXIT_CRITICAL(&gPacketMux);

  return hasPacket;
}

void processPacket(const ControlPacket &packet, const uint8_t senderMac[6]) {
  const bool isPing = packet.command == CMD_PING;
  const bool isKill = packet.command == CMD_KILL;

  if (gKillLatched && !isKill) {
    Serial.print("Rejected packet seq=");
    Serial.print(packet.seq);
    Serial.print(" cmd=");
    Serial.print(commandToString(packet.command));
    Serial.println(" reason=KILL_LATCHED");
    sendFeedbackPacket(senderMac, packet, FEEDBACK_REJECTED_KILL);
    return;
  }

  if (isKill) {
    portENTER_CRITICAL(&gPacketMux);
    gKillPendingProcessing = false;
    portEXIT_CRITICAL(&gPacketMux);
  }

  if (gLinkStats.hasSeenPacket && !isSequenceNewer(packet.seq, gLinkStats.lastAcceptedSeq) && !isPing && !isKill) {
    Serial.print("Ignoring stale/replayed packet seq=");
    Serial.println(packet.seq);
    sendFeedbackPacket(senderMac, packet, FEEDBACK_REJECTED_STALE);
    return;
  }

  gLinkStats.hasSeenPacket = true;
  gLinkStats.lastAcceptedSeq = packet.seq;

  uint8_t status = FEEDBACK_ACCEPTED;
  if (isDistanceCommand(packet.command) && packet.distanceCm > 0 && !hasGpsQualityForDistanceCommand(packet.command)) {
    status = FEEDBACK_GPS_FALLBACK;
  }

  if (!isPing) {
    gLinkStats.lastPacketAtMs = millis();
    applyCommand(packet);
  }

  Serial.print(isPing ? "Accepted/resynced packet seq=" : "Accepted packet seq=");
  Serial.print(packet.seq);
  Serial.print(" cmd=");
  Serial.print(commandToString(packet.command));
  Serial.print(" distanceCm=");
  Serial.println(packet.distanceCm);

  sendFeedbackPacket(senderMac, packet, status);
}

// ============================================================
// MARK: FAILSAFE LOGIC
// ============================================================

bool isArmedForFailsafe() {
  // AUX1 high is the project-wide arm signal.
  return targetChannels.aux1 > ReceiverConfig::kRcMid;
}

bool isLikelyGrounded() {
  // Keep a small margin above minimum throttle to avoid false in-flight matches.
  return currentChannels.throttle <= (ReceiverConfig::kRcMin + 25);
}

bool isCommandStillRunning() {
  // Keep timeout failsafes out of the way while a commanded move is still active.
  return gDistanceMove.active;
}

void handleFailsafeTimeouts() {
  if (gKillLatched || gDroneState == DroneState::KILL) {
  return;
  }
  if (!gLinkStats.hasSeenPacket) {
    return;
  }

  // Timeout failsafes are flight-only protections; skip while disarmed/idle/grounded.
  if (!isArmedForFailsafe() || gDroneState == DroneState::IDLE || isLikelyGrounded()) {
    resetAltitudeHold();
    return;
  }

  const uint32_t now = millis();
  const uint32_t elapsedSincePacket = now - gLinkStats.lastPacketAtMs;

  if (elapsedSincePacket >= gSafetyTuning.landFailsafeMs &&
      gDroneState != DroneState::LAND_FAILSAFE) {
    setThrottleTarget(makeDescentThrottle(gCommandTuning.powerStepPercent, gVehicleTuning.landingDropMultiplier));
    enterDroneState(DroneState::LAND_FAILSAFE);
    updateStateTargets();
    gLinkStats.lastLandingFailsafeAtMs = now;
    Serial.println("Landing failsafe triggered");
    return;
  }

  // Keep finite active commands running, but let the 30s land failsafe override them.
  if (isCommandStillRunning()) {
    return;
  }
  if (elapsedSincePacket >= gSafetyTuning.hoverFailsafeMs &&
      gDroneState != DroneState::HOVER_FAILSAFE &&
      gDroneState != DroneState::LAND_FAILSAFE) {
    enterDroneState(DroneState::HOVER_FAILSAFE);
    updateStateTargets();
    gLinkStats.lastHoverFailsafeAtMs = now;
    Serial.println("Hover failsafe triggered");
  }
}

// ============================================================
// MARK: OUTPUT UPDATE AND CRSF
// ============================================================

void updateCurrentChannels() {
  if (gKillLatched || gDroneState == DroneState::KILL) {
    setArmTarget(false);
    applyMotionTargets(cmdStop());
    currentChannels = targetChannels;
    return;
  }
  currentChannels.roll = slewTowards(currentChannels.roll, targetChannels.roll, gOutputSmoothing.axisSlewPerTick);
  currentChannels.pitch = slewTowards(currentChannels.pitch, targetChannels.pitch, gOutputSmoothing.axisSlewPerTick);
  currentChannels.yaw = slewTowards(currentChannels.yaw, targetChannels.yaw, gOutputSmoothing.axisSlewPerTick);
  currentChannels.throttle = slewTowards(currentChannels.throttle, targetChannels.throttle, gOutputSmoothing.throttleSlewPerTick);
  currentChannels.aux1 = targetChannels.aux1;
  currentChannels.aux2 = targetChannels.aux2;
  currentChannels.aux3 = targetChannels.aux3;
  currentChannels.aux4 = targetChannels.aux4;
}

void sendCrsfChannels(const RcChannels &ch) {
  uint16_t channels[16] = {
    rcUsToCrsf(ch.roll),
    rcUsToCrsf(ch.pitch),
    rcUsToCrsf(ch.throttle),
    rcUsToCrsf(ch.yaw),
    rcUsToCrsf(ch.aux1),
    rcUsToCrsf(ch.aux2),
    rcUsToCrsf(ch.aux3),
    rcUsToCrsf(ch.aux4),
    CrsfConfig::kCrsfMid,
    CrsfConfig::kCrsfMid,
    CrsfConfig::kCrsfMid,
    CrsfConfig::kCrsfMid,
    CrsfConfig::kCrsfMid,
    CrsfConfig::kCrsfMid,
    CrsfConfig::kCrsfMid,
    CrsfConfig::kCrsfMid,
  };

  uint8_t frame[CrsfConfig::kFrameSize] = {};
  frame[0] = CrsfConfig::kAddressFlightController;
  frame[1] = CrsfConfig::kFrameLength;
  frame[2] = CrsfConfig::kFrameTypeRcChannelsPacked;

  uint32_t bitBuffer = 0;
  uint8_t bitsInBuffer = 0;
  uint8_t payloadIndex = 3;
  for (uint8_t i = 0; i < 16; ++i) {
    bitBuffer |= static_cast<uint32_t>(channels[i] & 0x07FF) << bitsInBuffer;
    bitsInBuffer += 11;
    while (bitsInBuffer >= 8 && payloadIndex < 3 + CrsfConfig::kRcPayloadSize) {
      frame[payloadIndex++] = static_cast<uint8_t>(bitBuffer & 0xFF);
      bitBuffer >>= 8;
      bitsInBuffer -= 8;
    }
  }

  frame[CrsfConfig::kFrameSize - 1] = crc8DvbS2(&frame[2], CrsfConfig::kFrameLength - 1);
  Serial2.write(frame, sizeof(frame));
}

// ============================================================
// MARK: DEBUG HELPERS
// ============================================================

void printDebugStatus() {
  const uint32_t now = millis();
  if (now - gLastDebugPrintAtMs < ReceiverConfig::kDebugPrintIntervalMs) {
    return;
  }

  gLastDebugPrintAtMs = now;

  if (gSawInvalidPacketSize) {
    portENTER_CRITICAL(&gPacketMux);
    const int invalidLen = gLastInvalidPacketSize;
    gSawInvalidPacketSize = false;
    portEXIT_CRITICAL(&gPacketMux);

    Serial.print("Rejected packet with size ");
    Serial.print(invalidLen);
    Serial.print(" expected=");
    Serial.println(sizeof(ControlPacket));
  }

  Serial.print("packetAgeMs=");
  if (gLinkStats.hasSeenPacket) {
    Serial.print(now - gLinkStats.lastPacketAtMs);
  } else {
    Serial.print("N/A");
  }

  Serial.print(" State=");
  Serial.print(stateToString(gDroneState));
  Serial.print(" current[roll=");
  Serial.print(currentChannels.roll);
  Serial.print(" pitch=");
  Serial.print(currentChannels.pitch);
  Serial.print(" yaw=");
  Serial.print(currentChannels.yaw);
  Serial.print(" throttle=");
  Serial.print(currentChannels.throttle);
  Serial.print(" targetThrottle=");
  Serial.print(targetChannels.throttle);
  Serial.print(" crsfThrottle=");
  Serial.print(rcUsToCrsf(currentChannels.throttle));
  Serial.print(" aux1=");
  Serial.print(currentChannels.aux1);
  Serial.print("]");

  Serial.print(" gps[telemetryReceived=");
  Serial.print(hasRecentGpsTelemetry() ? "yes" : "no");
  Serial.print(" fixValid=");
  Serial.print(hasValidGpsFix() ? "yes" : "no");
  Serial.print(" horizontalQuality=");
  Serial.print(hasGpsQualityForHorizontalMovement() ? "yes" : "no");
  Serial.print(" altitudeQuality=");
  Serial.print(hasGpsQualityForAltitudeControl() ? "yes" : "no");
  Serial.print(" gpsQualitySource=");
  Serial.print(gpsQualitySourceForDebug());
  Serial.print(" latE7=");
  Serial.print(gGpsTelemetry.latitudeE7);
  Serial.print(" lonE7=");
  Serial.print(gGpsTelemetry.longitudeE7);
  Serial.print(" rawAltitude=");
  Serial.print(gGpsTelemetry.rawAltitude);
  Serial.print(" decodedAltitudeCm=");
  Serial.print(gGpsTelemetry.altitudeCm);
  Serial.print(" sats=");
  Serial.print(gGpsTelemetry.satellites);
  Serial.print(" fixType=");
  Serial.print(gGpsTelemetry.fixType);
  Serial.print(" hAcc=");
  Serial.print(gGpsTelemetry.horizontalAccuracyCm);
  Serial.print(" vAcc=");
  Serial.print(gGpsTelemetry.verticalAccuracyCm);
  Serial.print(" hDOP=");
  Serial.print(gGpsTelemetry.hDop10);
  Serial.print(" vDOP=");
  Serial.print(gGpsTelemetry.vDop10);
  Serial.print(" gpsAgeMs=");
  if (gGpsTelemetry.telemetryReceived) {
    Serial.print(now - gGpsTelemetry.lastTelemetryMs);
  } else {
    Serial.print("N/A");
  }
  Serial.print("]");

  Serial.print(" crsfRx[bytes=");
  Serial.print(gCrsfTelemetryStats.bytes);
  Serial.print(" frames=");
  Serial.print(gCrsfTelemetryStats.frames);
  Serial.print(" gps=");
  Serial.print(gCrsfTelemetryStats.gpsFrames);
  Serial.print(" gpsExt=");
  Serial.print(gCrsfTelemetryStats.gpsExtendedFrames);
  Serial.print(" crcErr=");
  Serial.print(gCrsfTelemetryStats.crcErrors);
  Serial.print(" sizeErr=");
  Serial.print(gCrsfTelemetryStats.sizeErrors);
  Serial.print(" byteAgeMs=");
  if (gCrsfTelemetryStats.lastByteAtMs > 0) {
    Serial.print(now - gCrsfTelemetryStats.lastByteAtMs);
  } else {
    Serial.print("N/A");
  }
  Serial.print(" gpsAgeMs=");
  if (gCrsfTelemetryStats.lastGpsAtMs > 0) {
    Serial.print(now - gCrsfTelemetryStats.lastGpsAtMs);
  } else {
    Serial.print("N/A");
  }
  Serial.print("]");

  if (gDistanceMove.active) {
    Serial.print(" distance[mode=");
    Serial.print(gDistanceMove.useTimeFallback ? "TIMED_FALLBACK" : "GPS_MEASURED");
    Serial.print(" targetCm=");
    Serial.print(gDistanceMove.targetDistanceCm);
    Serial.print(" traveledCm=");
    Serial.print(gDistanceMove.traveledDistanceCm);
    Serial.print(" profilePct=");
    Serial.print(computeDistanceProfileScalePercent(gDistanceMove.targetDistanceCm, gDistanceMove.traveledDistanceCm));
    Serial.print("]");
  }

  Serial.println();
}

// ============================================================
// MARK:ESP-NOW SETUP AND CALLBACK
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial2.begin(
      ReceiverConfig::kCrsfBaud,
      SERIAL_8N1,
      ReceiverConfig::kCrsfRxPin,
      ReceiverConfig::kCrsfTxPin);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ReceiverConfig::kEspNowChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  setNeutralTargets();
  setThrottleTarget(ReceiverConfig::kRcMin);
  currentChannels = targetChannels;
  gStateEnteredAtMs = millis();

  Serial.println("ESP-NOW initialized and receive callback registered");
  Serial.print("CRSF output on GPIO");
  Serial.print(ReceiverConfig::kCrsfTxPin);
  Serial.print(" at ");
  Serial.print(ReceiverConfig::kCrsfBaud);
  Serial.println(" baud");
  Serial.print("CRSF telemetry input on GPIO");
  Serial.println(ReceiverConfig::kCrsfRxPin);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != static_cast<int>(sizeof(ControlPacket))) {
    portENTER_CRITICAL_ISR(&gPacketMux);
    gLastInvalidPacketSize = len;
    gSawInvalidPacketSize = true;
    portEXIT_CRITICAL_ISR(&gPacketMux);
    return;
  }

  ControlPacket receivedPacket{};
  memcpy(&receivedPacket, data, sizeof(ControlPacket));

  portENTER_CRITICAL_ISR(&gPacketMux);
  if (receivedPacket.command == CMD_KILL) {
    gKillLatched = true;
    gKillPendingProcessing = true;
    memcpy(&gPendingPacket, &receivedPacket, sizeof(ControlPacket));
    memcpy(gPendingSenderMac, mac, 6);
    gHasPendingPacket = true;
  } else if (!gKillLatched || !gKillPendingProcessing) {
    memcpy(&gPendingPacket, &receivedPacket, sizeof(ControlPacket));
    memcpy(gPendingSenderMac, mac, 6);
    gHasPendingPacket = true;
  }
  portEXIT_CRITICAL_ISR(&gPacketMux);
}

// ============================================================
// MARK: MAIN LOOP
// ============================================================

void loop() {
  ControlPacket packet;
  uint8_t senderMac[6] = {};
  if (fetchPendingPacket(packet, senderMac)) {
    processPacket(packet, senderMac);
  }

  readCrsfTelemetry();
  handleFailsafeTimeouts();
  updateStateTargets();
  updateCurrentChannels();
  sendCrsfChannels(currentChannels);
  printDebugStatus();

  delay(ReceiverConfig::kLoopDelayMs);
}

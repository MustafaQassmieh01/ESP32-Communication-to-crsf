// ============================================================
// MARK: INCLUDES
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ============================================================
// MARK: CONFIGURATION CONSTANTS
// ============================================================

namespace ReceiverConfig {
constexpr uint8_t kEspNowChannel = 1;

constexpr uint16_t kRcMin = 1000;
constexpr uint16_t kRcMid = 1500;
constexpr uint16_t kRcMax = 2000;

constexpr uint8_t kDefaultPowerStepPercent = 12;
constexpr uint8_t kDefaultAngleStepPercent = 18;

constexpr uint32_t kHoverFailsafeMs = 7000; // This is the number of seconds without a command before the drone should enter hover failsafe mode.
constexpr uint32_t kLandFailsafeMs = 30000; // This is the number of seconds without a command before the drone should enter landing failsafe mode.(should start after hover failsafe)
constexpr uint32_t kLoopDelayMs = 20;
constexpr uint32_t kDebugPrintIntervalMs = 500;
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
  CMD_KILL // failsafe command to immediately cut throttle, bypassing smoothing and safety checks. Should be used in emergencies only.
};

enum class DroneState : uint8_t {
  IDLE = 0,
  TAKEOFF,
  ACTIVE,
  HOVER_FAILSAFE,
  LAND_FAILSAFE,
  KILL
};

struct __attribute__((packed)) ControlPacket {
  uint32_t seq;
  uint8_t command;
  int16_t value;
  uint16_t durationMs;
};

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
};

// ============================================================
// MARK: GLOBAL STATE
// ============================================================

volatile bool gHasPendingPacket = false;
volatile bool gSawInvalidPacketSize = false;
volatile bool gKillLatched = false; // Latched emergency stop. Cleared only by receiver reset.
volatile int gLastInvalidPacketSize = 0;
ControlPacket gPendingPacket = {};
portMUX_TYPE gPacketMux = portMUX_INITIALIZER_UNLOCKED;

LinkStats gLinkStats;
DroneState gDroneState = DroneState::IDLE;

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

void setNeutralTargets() {
  targetChannels.roll = ReceiverConfig::kRcMid;
  targetChannels.pitch = ReceiverConfig::kRcMid;
  targetChannels.yaw = ReceiverConfig::kRcMid;
}

void setThrottleTarget(uint16_t throttle) {
  targetChannels.throttle = clampRc(throttle);
}

void setHoverThrottleTarget() {
  setThrottleTarget(gVehicleTuning.hoverThrottle);
}

uint16_t makeThrottleFromPercent(int percent) {
  return percentToThrottle(percent);
}

uint16_t makeLiftThrottle(int percent, float multiplier) {
  return percentToThrottle(static_cast<int>(percent * multiplier));
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
    default: return "UNKNOWN_CMD";
  }
}
// ============================================================
// MARK: COMMAND BUILDERS
// ============================================================

MotionTargets cmdStop() {
  MotionTargets motion;
  motion.useHoverThrottle = false;
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
  motion.throttlePercent = static_cast<int8_t>(gCommandTuning.powerStepPercent * gVehicleTuning.landingDropMultiplier);
  return motion;
}

void applyMotionTargets(const MotionTargets &motion) {
  setNeutralTargets();
  targetChannels.pitch = makePitchTarget(motion.pitchPercent);
  targetChannels.roll = makeRollTarget(motion.rollPercent);
  targetChannels.yaw = makeYawTarget(motion.yawPercent);

  if (motion.useHoverThrottle) {
    setHoverThrottleTarget();
  } else {
    setThrottleTarget(makeThrottleFromPercent(motion.throttlePercent));
  }
}

// ============================================================
// MARK: STATE TARGET UPDATE LOGIC
// ============================================================

void updateStateTargets() {
  switch (gDroneState) {
    case DroneState::IDLE:
      applyMotionTargets(cmdStop());
      break;
    case DroneState::TAKEOFF: {
      MotionTargets motion = cmdHover();
      motion.useHoverThrottle = false;
      motion.throttlePercent = static_cast<int8_t>(
          gCommandTuning.powerStepPercent * gVehicleTuning.takeoffLiftMultiplier);
      applyMotionTargets(motion);
      break;
    }
    case DroneState::ACTIVE:
      // Active state is driven directly by incoming commands, so no need to apply any default targets here.
      break;
    case DroneState::HOVER_FAILSAFE:
      applyMotionTargets(cmdHover());
      break;
    case DroneState::LAND_FAILSAFE: {
      MotionTargets motion = cmdHover();
      motion.useHoverThrottle = false;
      motion.throttlePercent = static_cast<int8_t>(
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
    gKillLatched = true;
    gDroneState = DroneState::KILL;
    motion = cmdStop();
    applyMotionTargets(motion);
    return;
  }

  if (gKillLatched) {
    return;
  }
  switch (packet.command) {
    case CMD_STOP:
    case CMD_DISARM:
      gDroneState = DroneState::IDLE;
      motion = cmdStop();
      break;
    case CMD_ARM:
    case CMD_HOVER:
      gDroneState = DroneState::ACTIVE;
      motion = cmdHover();
      break;
    case CMD_TAKEOFF:
      gDroneState = DroneState::TAKEOFF;
      applyMotion = false;
      break;
    case CMD_LAND:
      gDroneState = DroneState::LAND_FAILSAFE;
      applyMotion = false;
      break;
    case CMD_FORWARD:
      gDroneState = DroneState::ACTIVE;
      motion = cmdForward();
      break;
    case CMD_BACK:
      gDroneState = DroneState::ACTIVE;
      motion = cmdBack();
      break;
    case CMD_LEFT:
      gDroneState = DroneState::ACTIVE;
      motion = cmdLeft();
      break;
    case CMD_RIGHT:
      gDroneState = DroneState::ACTIVE;
      motion = cmdRight();
      break;
    case CMD_YAW_LEFT:
      gDroneState = DroneState::ACTIVE;
      motion = cmdYawLeft();
      break;
    case CMD_YAW_RIGHT:
      gDroneState = DroneState::ACTIVE;
      motion = cmdYawRight();
      break;
    case CMD_UP:
      gDroneState = DroneState::ACTIVE;
      motion = cmdUp();
      if (packet.value != 0) {
        motion.throttlePercent = static_cast<int8_t>(packet.value);
      }
      break;
    case CMD_DOWN:
      gDroneState = DroneState::ACTIVE;
      motion = cmdDown();
      if (packet.value != 0) {
        motion.throttlePercent = static_cast<int8_t>(packet.value * gVehicleTuning.landingDropMultiplier);
      }
      break;
    default:
      gDroneState = DroneState::ACTIVE;
      motion = cmdHover();
      break;
  }

  if (applyMotion) {
    applyMotionTargets(motion);
  } else {
    updateStateTargets();
  }

  // Duration support is intentionally deferred. The field is accepted and
  // carried through the packet pipeline so timed actions can be added later.
  (void)packet.durationMs;
}

// ============================================================
// MARK: PACKET HANDLING
// ============================================================

bool isSequenceNewer(uint32_t seq, uint32_t reference) {
  return static_cast<int32_t>(seq - reference) > 0;
}

bool fetchPendingPacket(ControlPacket &packetOut) {
  bool hasPacket = false;

  portENTER_CRITICAL(&gPacketMux);
  if (gHasPendingPacket) {
    packetOut = gPendingPacket;
    gHasPendingPacket = false;
    hasPacket = true;
  }
  portEXIT_CRITICAL(&gPacketMux);

  return hasPacket;
}

void processPacket(const ControlPacket &packet) {
  if (gLinkStats.hasSeenPacket && !isSequenceNewer(packet.seq, gLinkStats.lastAcceptedSeq)) {
    Serial.print("Ignoring stale/replayed packet seq=");
    Serial.println(packet.seq);
    return;
  }

  gLinkStats.hasSeenPacket = true;
  gLinkStats.lastAcceptedSeq = packet.seq;
  gLinkStats.lastPacketAtMs = millis();

  applyCommand(packet);

Serial.print("Accepted packet seq=");
Serial.print(packet.seq);
Serial.print(" cmd=");
Serial.print(commandToString(packet.command));
Serial.print(" value=");
Serial.print(packet.value);
Serial.print(" durationMs=");
Serial.println(packet.durationMs);
}

// ============================================================
// MARK: FAILSAFE LOGIC
// ============================================================

void handleFailsafeTimeouts() {
  if (gKillLatched || gDroneState == DroneState::KILL) {
  return;
  }
  if (!gLinkStats.hasSeenPacket) {
    return;
  }

  const uint32_t now = millis();
  const uint32_t elapsedSincePacket = now - gLinkStats.lastPacketAtMs;

  if (elapsedSincePacket >= gSafetyTuning.landFailsafeMs &&
      gDroneState != DroneState::LAND_FAILSAFE) {
    setThrottleTarget(makeLiftThrottle(gCommandTuning.powerStepPercent, gVehicleTuning.landingDropMultiplier));
    gDroneState = DroneState::LAND_FAILSAFE;
    updateStateTargets();
    gLinkStats.lastLandingFailsafeAtMs = now;
    Serial.println("Landing failsafe placeholder triggered");
    return;
  }

  if (elapsedSincePacket >= gSafetyTuning.hoverFailsafeMs &&
      gDroneState != DroneState::HOVER_FAILSAFE &&
      gDroneState != DroneState::LAND_FAILSAFE) {
    gDroneState = DroneState::HOVER_FAILSAFE;
    updateStateTargets();
    gLinkStats.lastHoverFailsafeAtMs = now;
    Serial.println("Hover failsafe placeholder triggered");
  }
}

// ============================================================
// MARK: OUTPUT UPDATE AND CRSF PLACEHOLDER
// ============================================================

void updateCurrentChannels() {
  if (gKillLatched || gDroneState == DroneState::KILL) {
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
  // Placeholder for future CRSF encoding/output. Keep behavior logic isolated
  // from protocol-specific serialization until the CRSF module is added.
  (void)ch;
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
    Serial.println(invalidLen);
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
  Serial.println("]");
}

// ============================================================
// MARK:ESP-NOW SETUP AND CALLBACK
// ============================================================

void setup() {
  Serial.begin(115200);
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

  Serial.println("ESP-NOW initialized and receive callback registered");
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;

  if (len != static_cast<int>(sizeof(ControlPacket))) {
    portENTER_CRITICAL_ISR(&gPacketMux);
    gLastInvalidPacketSize = len;
    gSawInvalidPacketSize = true;
    portEXIT_CRITICAL_ISR(&gPacketMux);
    return;
  }

  portENTER_CRITICAL_ISR(&gPacketMux);
  memcpy(&gPendingPacket, data, sizeof(ControlPacket));
  gHasPendingPacket = true;
  portEXIT_CRITICAL_ISR(&gPacketMux);
}

// ============================================================
// MARK: MAIN LOOP
// ============================================================

void loop() {
  ControlPacket packet;
  if (fetchPendingPacket(packet)) {
    processPacket(packet);
  }

  handleFailsafeTimeouts();
  updateStateTargets();
  updateCurrentChannels();
  sendCrsfChannels(currentChannels);
  printDebugStatus();

  delay(ReceiverConfig::kLoopDelayMs);
}

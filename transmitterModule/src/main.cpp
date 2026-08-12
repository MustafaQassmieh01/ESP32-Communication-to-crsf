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

namespace SenderConfig {
constexpr uint8_t kEspNowChannel = 1;
constexpr uint32_t kSerialBaud = 115200;
constexpr size_t kInputBufferSize = 128;
constexpr size_t kSentTimingSlots = 8;
}

// ============================================================
// MARK: SHARED PACKET TYPES
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
  CMD_KILL,
  CMD_PING
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

struct SentPacketTiming {
  bool active = false;
  uint32_t seq = 0;
  uint32_t sentAtMs = 0;
};

// ============================================================
// MARK: GLOBAL STATE
// ============================================================

// Mac address of the drone receiver ESP32-U.
uint8_t gReceiverMac[] = {0xD4, 0xE9, 0xF4, 0xE1, 0xC5, 0x78};

uint32_t gNextSeq = 1;
char gInputBuffer[SenderConfig::kInputBufferSize];
size_t gInputPos = 0;
SentPacketTiming gSentTimings[SenderConfig::kSentTimingSlots];
volatile bool gNeedsResyncPing = false;
volatile bool gAwaitingResyncAck = false;
volatile bool gResyncAcked = false;
volatile bool gHasRetryAfterResync = false;
ControlPacket gRetryAfterResyncPacket = {};
portMUX_TYPE gResyncMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// MARK: DEBUG HELPERS
// ============================================================

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

void printMacAddress(const uint8_t* mac) {
  for (int i = 0; i < 6; ++i) {
    if (i > 0) {
      Serial.print(":");
    }
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
  }
}

const char* feedbackStatusToString(uint8_t status) {
  switch (status) {
    case FEEDBACK_ACCEPTED: return "ACCEPTED";
    case FEEDBACK_REJECTED_STALE: return "REJECTED_STALE";
    case FEEDBACK_REJECTED_GPS: return "REJECTED_GPS";
    case FEEDBACK_GPS_FALLBACK: return "GPS_FALLBACK";
    case FEEDBACK_INVALID: return "INVALID";
    case FEEDBACK_REJECTED_KILL: return "REJECTED_KILL";
    default: return "UNKNOWN";
  }
}

void recordSentPacket(uint32_t seq) {
  const size_t slot = seq % SenderConfig::kSentTimingSlots;
  gSentTimings[slot].active = true;
  gSentTimings[slot].seq = seq;
  gSentTimings[slot].sentAtMs = millis();
}

bool consumeSentPacketTime(uint32_t seq, uint32_t& sentAtMs) {
  const size_t slot = seq % SenderConfig::kSentTimingSlots;
  if (!gSentTimings[slot].active || gSentTimings[slot].seq != seq) {
    return false;
  }

  sentAtMs = gSentTimings[slot].sentAtMs;
  gSentTimings[slot].active = false;
  return true;
}
// ============================================================
// MARK: STRING HELPERS
// ============================================================

String trimAndUppercase(String input) {
  input.trim();
  input.toUpperCase();
  return input;
}

bool startsWithIgnoreCase(const String& input, const String& prefix) {
  String tempInput = input;
  String tempPrefix = prefix;
  tempInput.toUpperCase();
  tempPrefix.toUpperCase();
  return tempInput.startsWith(tempPrefix);
}
bool matchesCommandToken(const String& input, const String& command) {
  if (!startsWithIgnoreCase(input, command)) {
    return false;
  }

  if (input.length() == command.length()) {
    return true;
  }

  const char separator = input.charAt(command.length());
  return separator == ' ' || separator == ':' || separator == '\t';
}

// ============================================================
// MARK: PARSING
// ============================================================

bool isValidFloatToken(const String& token, bool allowSign) {
  if (token.length() == 0) {
    return false;
  }

  bool hasDigit = false;
  bool hasDecimalPoint = false;
  for (size_t i = 0; i < token.length(); ++i) {
    const char c = token.charAt(i);
    if ((c == '+' || c == '-') && allowSign && i == 0) {
      continue;
    }
    if (c >= '0' && c <= '9') {
      hasDigit = true;
      continue;
    }
    if (c == '.' && !hasDecimalPoint) {
      hasDecimalPoint = true;
      continue;
    }
    return false;
  }

  return hasDigit;
}

bool tryParseFloatSuffix(const String& input, const String& prefix, float& outValue) {
  if (!startsWithIgnoreCase(input, prefix)) {
    return false;
  }

  String suffix = input.substring(prefix.length());
  suffix.trim();

  if (!isValidFloatToken(suffix, true)) {
    Serial.print("Invalid numeric value for ");
    Serial.print(prefix);
    Serial.print(" ");
    Serial.println(suffix);
    return false;
  }

  outValue = suffix.toFloat();
  return true;
}

bool isValidUnsignedNumberToken(const String& token) {
  return isValidFloatToken(token, false);
}

bool tryParseDistanceToken(const String& token, uint16_t& outDistanceCm) {
  if (!isValidUnsignedNumberToken(token)) {
    return false;
  }

  const float distanceCm = token.toFloat();
  if (distanceCm < 0.0f) {
    return false;
  }
  if (distanceCm > 65535.0f) {
    outDistanceCm = 65535;
    return true;
  }

  outDistanceCm = static_cast<uint16_t>(distanceCm);
  return true;
}

bool parseDistanceCm(const String& input, const String& command, uint16_t& outDistanceCm) {
  if (!matchesCommandToken(input, command)) {
    return false;
  }

  String suffix = input.substring(command.length());
  suffix.trim();

  if (suffix.startsWith(":")) {
    suffix = suffix.substring(1);
    suffix.trim();
  }

  outDistanceCm = 0;
  if (suffix.length() == 0) {
    return true;
  }

  const int separator = suffix.indexOf(' ');
  String distanceToken = separator >= 0 ? suffix.substring(0, separator) : suffix;
  distanceToken.trim();

  if (!tryParseDistanceToken(distanceToken, outDistanceCm)) {
    Serial.print("Invalid distance for ");
    Serial.print(command);
    Serial.print(": ");
    Serial.println(distanceToken);
    return false;
  }

  return true;
}

bool parseMovementCommand(const String& input, const String& shortCommand, const String& moveCommand, uint8_t command, ControlPacket& outPacket) {
  uint16_t distanceCm = 0;
  if (parseDistanceCm(input, shortCommand, distanceCm) || parseDistanceCm(input, moveCommand, distanceCm)) {
    outPacket.command = command;
    outPacket.distanceCm = distanceCm;
    return true;
  }

  return false;
}
ControlPacket makeDefaultPacket() {
  ControlPacket packet{};
  packet.seq = gNextSeq++;
  packet.command = CMD_HOVER;
  packet.distanceCm = 0;
  return packet;
}

bool parseInputToPacket(const String& rawInput, ControlPacket& outPacket) {
  String input = trimAndUppercase(rawInput);

  if (input.length() == 0) {
    return false;
  }

  outPacket = makeDefaultPacket();

  // ----------------------------------------------------------
  // SYSTEM COMMANDS
  // ----------------------------------------------------------

  if (matchesCommandToken(input, "START") || matchesCommandToken(input, "ARM") || matchesCommandToken(input, "ARISE")) {
    outPacket.command = CMD_ARM;
    return true;
  }

  if (matchesCommandToken(input, "DISARM")) {
    outPacket.command = CMD_DISARM;
    return true;
  }

  if (matchesCommandToken(input, "TAKE_OFF") || matchesCommandToken(input, "TAKEOFF")) {
    outPacket.command = CMD_TAKEOFF;
    return true;
  }

  if (matchesCommandToken(input, "LAND")) {
    outPacket.command = CMD_LAND;
    return true;
  }

  if (matchesCommandToken(input, "HOVER") || matchesCommandToken(input, "STOP") || matchesCommandToken(input, "FREEZE") || matchesCommandToken(input, "STAY") || matchesCommandToken(input, "HOLD") || matchesCommandToken(input, "STEADY")) {
    outPacket.command = CMD_HOVER;
    return true;
  }

  if (matchesCommandToken(input, "KILL")) {
    outPacket.command = CMD_KILL;
    return true;
  }

  if (matchesCommandToken(input, "PING")) {
    outPacket.command = CMD_PING;
    return true;
  }

  // ----------------------------------------------------------
  // MOVEMENT COMMANDS
  // ----------------------------------------------------------

  if (parseMovementCommand(input, "FORWARD", "MOVE_FORWARD", CMD_FORWARD, outPacket)) {
    return true;
  }

  if (parseMovementCommand(input, "BACK", "MOVE_BACK", CMD_BACK, outPacket)) {
    return true;
  }

  if (parseMovementCommand(input, "LEFT", "MOVE_LEFT", CMD_LEFT, outPacket)) {
    return true;
  }

  if (parseMovementCommand(input, "RIGHT", "MOVE_RIGHT", CMD_RIGHT, outPacket)) {
    return true;
  }

  if (parseMovementCommand(input, "UP", "MOVE_UP", CMD_UP, outPacket)) {
    return true;
  }

  if (parseMovementCommand(input, "DOWN", "MOVE_DOWN", CMD_DOWN, outPacket)) {
    return true;
  }

  if (matchesCommandToken(input, "YAW_LEFT") || matchesCommandToken(input, "TURN_LEFT")) {
    outPacket.command = CMD_YAW_LEFT;
    return true;
  }

  if (matchesCommandToken(input, "YAW_RIGHT") || matchesCommandToken(input, "TURN_RIGHT")) {
    outPacket.command = CMD_YAW_RIGHT;
    return true;
  }

  // ----------------------------------------------------------
  // LEGACY THROTTLE STRING: T:x.x
  // With distance-based commands, only T:0.0 remains mapped as a kill.
  // ----------------------------------------------------------

  float throttleValue = 0.0f;
  if (tryParseFloatSuffix(input, "T:", throttleValue)) {
    if (throttleValue <= 0.01f) {
      outPacket.command = CMD_KILL;
      return true;
    }

    Serial.println("Throttle value input is disabled; use UP <distanceCm> or DOWN <distanceCm>.");
    return false;
  }

  // ----------------------------------------------------------
  // PITCH FREQUENCY STRING: P:x
  // Example: P:210
  // For now: parse and log only, map later if needed.
  // ----------------------------------------------------------

  float pitchHz = 0.0f;
  if (tryParseFloatSuffix(input, "P:", pitchHz)) {
    Serial.print("Received pitch frequency input, not mapped yet: ");
    Serial.println(pitchHz);
    return false;
  }

  Serial.print("Unknown input: ");
  Serial.println(rawInput);
  return false;
}

// ============================================================
// MARK: ESP-NOW
// ============================================================

void onPacketSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    return;
  }

  Serial.print("TX_FAIL mac=");
  printMacAddress(mac_addr);
  Serial.println();
}
void onFeedbackReceived(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;

  if (len != static_cast<int>(sizeof(FeedbackPacket))) {
    Serial.print("ACK invalidSize=");
    Serial.print(len);
    Serial.print(" expected=");
    Serial.println(sizeof(FeedbackPacket));
    return;
  }

  FeedbackPacket feedback{};
  memcpy(&feedback, data, sizeof(feedback));

  uint32_t sentAtMs = 0;
  const bool hasTiming = consumeSentPacketTime(feedback.seq, sentAtMs);
  const uint32_t latencyMs = hasTiming ? millis() - sentAtMs : 0;

  Serial.print("ACK#");
  Serial.print(feedback.seq);
  Serial.print(" ");
  Serial.print(commandToString(feedback.command));
  Serial.print(" ");
  Serial.print(feedbackStatusToString(feedback.status));
  Serial.print(" state=");
  Serial.print(feedback.state);
  Serial.print(" ");
  if (hasTiming) {
    Serial.print(latencyMs);
  } else {
    Serial.print("?");
  }
  Serial.print("ms d=");
  Serial.print(feedback.distanceCm);
  Serial.print(" p=");
  Serial.print(feedback.progressCm);
  Serial.print(" thr=");
  Serial.print(feedback.throttleUs);
  Serial.print("/");
  Serial.print(feedback.targetThrottleUs);
  Serial.print(" c=");
  Serial.println(feedback.throttleCrsf);

  if (feedback.status == FEEDBACK_REJECTED_STALE) {
    if (feedback.command != CMD_PING) {
      portENTER_CRITICAL(&gResyncMux);
      gRetryAfterResyncPacket.command = feedback.command;
      gRetryAfterResyncPacket.distanceCm = feedback.distanceCm;
      gHasRetryAfterResync = true;
      portEXIT_CRITICAL(&gResyncMux);
    }
    gNeedsResyncPing = true;
  } else if (gAwaitingResyncAck && feedback.command == CMD_PING && feedback.status == FEEDBACK_ACCEPTED) {
    gAwaitingResyncAck = false;
    gResyncAcked = true;
  }
}
bool addReceiverPeer() {
  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, gReceiverMac, 6);
  peerInfo.channel = SenderConfig::kEspNowChannel;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_is_peer_exist(gReceiverMac)) {
    return true;
  }

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK) {
    Serial.print("Failed to add peer, error=");
    Serial.println(result);
    return false;
  }

  return true;
}

bool sendPacket(const ControlPacket& packet) {
  recordSentPacket(packet.seq);

  esp_err_t result = esp_now_send(
      gReceiverMac,
      reinterpret_cast<const uint8_t*>(&packet),
      sizeof(packet));

  if (result != ESP_OK) {
    uint32_t ignoredSentAt = 0;
    consumeSentPacketTime(packet.seq, ignoredSentAt);
    Serial.print("esp_now_send failed, error=");
    Serial.println(result);
    return false;
  }

  Serial.print("TX#");
  Serial.print(packet.seq);
  Serial.print(" ");
  Serial.print(commandToString(packet.command));
  Serial.print(" d=");
  Serial.println(packet.distanceCm);

  return true;
}

void sendStartupPing() {
  ControlPacket packet = makeDefaultPacket();
  packet.command = CMD_PING;
  packet.distanceCm = 0;
  sendPacket(packet);
}

void retryCommandAfterResync() {
  ControlPacket pendingRetry{};
  bool hasRetry = false;

  portENTER_CRITICAL(&gResyncMux);
  if (gHasRetryAfterResync) {
    pendingRetry = gRetryAfterResyncPacket;
    gHasRetryAfterResync = false;
    hasRetry = true;
  }
  portEXIT_CRITICAL(&gResyncMux);

  if (!hasRetry) {
    return;
  }

  ControlPacket retryPacket = makeDefaultPacket();
  retryPacket.command = pendingRetry.command;
  retryPacket.distanceCm = pendingRetry.distanceCm;

  Serial.print("Retry after resync: ");
  Serial.println(commandToString(retryPacket.command));
  sendPacket(retryPacket);
}
// ============================================================
// MARK: SERIAL INPUT
// ============================================================

void handleCompletedLine(const String& line) {
  ControlPacket packet{};
  if (!parseInputToPacket(line, packet)) {
    return;
  }

  sendPacket(packet);
}

void readSerialInput() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      gInputBuffer[gInputPos] = '\0';
      String line = String(gInputBuffer);
      gInputPos = 0;
      handleCompletedLine(line);
      continue;
    }

    if (gInputPos < SenderConfig::kInputBufferSize - 1) {
      gInputBuffer[gInputPos++] = c;
    }
  }
}

void handleResyncPing() {
  if (gResyncAcked) {
    gResyncAcked = false;
    retryCommandAfterResync();
  }

  if (!gNeedsResyncPing || gAwaitingResyncAck) {
    return;
  }

  gNeedsResyncPing = false;
  gAwaitingResyncAck = true;
  sendStartupPing();
}
// ============================================================
// MARK: SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(SenderConfig::kSerialBaud);
  delay(1000);

  Serial.println();
  Serial.println("TX boot");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(SenderConfig::kEspNowChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onPacketSent);
  esp_now_register_recv_cb(onFeedbackReceived);

  if (!addReceiverPeer()) {
    Serial.println("Failed to configure receiver peer");
    return;
  }

  Serial.print("MAC ");
  Serial.print(WiFi.macAddress());
  Serial.print(" > ");
  printMacAddress(gReceiverMac);
  Serial.println();

  sendStartupPing();
  Serial.println("Ready");
}

void loop() {
  handleResyncPing();
  readSerialInput();
}

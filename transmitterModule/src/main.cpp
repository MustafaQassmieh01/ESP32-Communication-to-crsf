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
  uint16_t durationMs;
} __attribute__((packed));

enum FeedbackStatus : uint8_t {
  FEEDBACK_ACCEPTED = 0,
  FEEDBACK_REJECTED_STALE,
  FEEDBACK_REJECTED_GPS,
  FEEDBACK_INVALID
};

struct FeedbackPacket {
  uint32_t seq;
  uint8_t command;
  uint8_t status;
  uint8_t state;
  uint16_t distanceCm;
  uint16_t progressCm;
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
    case FEEDBACK_INVALID: return "INVALID";
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

// ============================================================
// MARK: PARSING
// ============================================================

bool tryParseFloatSuffix(const String& input, const String& prefix, float& outValue) {
  if (!startsWithIgnoreCase(input, prefix)) {
    return false;
  }

  String suffix = input.substring(prefix.length());
  suffix.trim();

  if (suffix.length() == 0) {
    return false;
  }

  outValue = suffix.toFloat();
  return true;
}



bool parseDistanceCm(const String& input, const String& command, uint16_t& outDistanceCm) {
  if (!startsWithIgnoreCase(input, command)) {
    return false;
  }

  String suffix = input.substring(command.length());
  suffix.trim();

  if (suffix.startsWith(":")) {
    suffix = suffix.substring(1);
    suffix.trim();
  }

  if (suffix.length() == 0) {
    outDistanceCm = 0;
    return true;
  }

  float distanceCm = suffix.toFloat();
  if (distanceCm < 0.0f) {
    distanceCm = 0.0f;
  }
  if (distanceCm > 65535.0f) {
    distanceCm = 65535.0f;
  }

  outDistanceCm = static_cast<uint16_t>(distanceCm);
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
  packet.durationMs = 0;
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

  if (input == "START" || input == "ARM" || input == "ARISE") {
    outPacket.command = CMD_ARM;
    return true;
  }

  if (input == "STOP" || input == "DISARM") {
    outPacket.command = CMD_STOP;
    return true;
  }

  if (input == "TAKE_OFF" || input == "TAKEOFF") {
    outPacket.command = CMD_TAKEOFF;
    return true;
  }

  if (input == "LAND") {
    outPacket.command = CMD_LAND;
    return true;
  }

  if (input == "HOVER" || input == "FREEZE" || input == "STAY") {
    outPacket.command = CMD_HOVER;
    return true;
  }

  if (input == "KILL") {
    outPacket.command = CMD_KILL;
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

  if (input == "YAW_LEFT" || input == "TURN_LEFT") {
    outPacket.command = CMD_YAW_LEFT;
    return true;
  }

  if (input == "YAW_RIGHT" || input == "TURN_RIGHT") {
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
  Serial.print("Send callback to ");
  printMacAddress(mac_addr);
  Serial.print(" status=");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
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

  Serial.print("ACK seq=");
  Serial.print(feedback.seq);
  Serial.print(" cmd=");
  Serial.print(commandToString(feedback.command));
  Serial.print(" status=");
  Serial.print(feedbackStatusToString(feedback.status));
  Serial.print(" latencyMs=");
  if (hasTiming) {
    Serial.print(latencyMs);
  } else {
    Serial.print("N/A");
  }
  Serial.print(" distanceCm=");
  Serial.print(feedback.distanceCm);
  Serial.print(" progressCm=");
  Serial.print(feedback.progressCm);
  Serial.print(" receiverMs=");
  Serial.println(feedback.receiverMillis);
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

  Serial.print("Sent packet seq=");
  Serial.print(packet.seq);
  Serial.print(" cmd=");
  Serial.print(commandToString(packet.command));
  Serial.print(" distanceCm=");
  Serial.print(packet.distanceCm);
  Serial.print(" durationMs=");
  Serial.println(packet.durationMs);

  return true;
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

// ============================================================
// MARK: SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(SenderConfig::kSerialBaud);
  delay(1000);

  Serial.println();
  Serial.println("Sender booting...");

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

  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Receiver MAC: ");
  printMacAddress(gReceiverMac);
  Serial.println();

  Serial.println("Ready. Type commands like:");
  Serial.println("ARISE");
  Serial.println("MOVE_FORWARD 200");
  Serial.println("MOVE_LEFT:100");
  Serial.println("TURN_RIGHT");
  Serial.println("TURN_LEFT");
  Serial.println("PING");
  Serial.println("HOVER");
  Serial.println("LAND");
  Serial.println("KILL");
}

void loop() {
  readSerialInput();
}

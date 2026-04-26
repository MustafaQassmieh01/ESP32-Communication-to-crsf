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
  CMD_KILL
};

struct __attribute__((packed)) ControlPacket {
  uint32_t seq;
  uint8_t command;
  int16_t value;
  uint16_t durationMs;
};

// ============================================================
// MARK: GLOBAL STATE
// ============================================================

// Mac address of the receiver ESP32D No 1 (the drone).
uint8_t gReceiverMac[] = {0x14, 0x2B, 0x2F, 0xD8, 0xF8, 0x54};

uint32_t gNextSeq = 1;
char gInputBuffer[SenderConfig::kInputBufferSize];
size_t gInputPos = 0;

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

ControlPacket makeDefaultPacket() {
  ControlPacket packet{};
  packet.seq = gNextSeq++;
  packet.command = CMD_HOVER;
  packet.value = 0;
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

  if (input == "START" || input == "ARM") {
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

  // ----------------------------------------------------------
  // MOVEMENT COMMANDS
  // ----------------------------------------------------------

  if (input == "MOVE_FORWARD" || input == "FORWARD") {
    outPacket.command = CMD_FORWARD;
    return true;
  }

  if (input == "MOVE_BACK" || input == "BACK") {
    outPacket.command = CMD_BACK;
    return true;
  }

  if (input == "MOVE_LEFT" || input == "LEFT") {
    outPacket.command = CMD_LEFT;
    return true;
  }

  if (input == "MOVE_RIGHT" || input == "RIGHT") {
    outPacket.command = CMD_RIGHT;
    return true;
  }

  if (input == "MOVE_UP" || input == "UP") {
    outPacket.command = CMD_UP;
    return true;
  }

  if (input == "MOVE_DOWN" || input == "DOWN") {
    outPacket.command = CMD_DOWN;
    return true;
  }

  if (input == "YAW_LEFT") {
    outPacket.command = CMD_YAW_LEFT;
    return true;
  }

  if (input == "YAW_RIGHT") {
    outPacket.command = CMD_YAW_RIGHT;
    return true;
  }

  // ----------------------------------------------------------
  // THROTTLE STRING: T:x.x
  // Example: T:0.2
  // ----------------------------------------------------------

  float throttleValue = 0.0f;
  if (tryParseFloatSuffix(input, "T:", throttleValue)) {
    if (throttleValue <= 0.01f) {
      outPacket.command = CMD_KILL;
      outPacket.value = 0;
      return true;
    }

    int scaled = static_cast<int>(throttleValue * 100.0f);
    scaled = constrain(scaled, 0, 100);

    outPacket.command = CMD_UP;
    outPacket.value = scaled;
    return true;
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
  esp_err_t result = esp_now_send(
      gReceiverMac,
      reinterpret_cast<const uint8_t*>(&packet),
      sizeof(packet));

  if (result != ESP_OK) {
    Serial.print("esp_now_send failed, error=");
    Serial.println(result);
    return false;
  }

  Serial.print("Sent packet seq=");
  Serial.print(packet.seq);
  Serial.print(" cmd=");
  Serial.print(commandToString(packet.command));
  Serial.print(" value=");
  Serial.print(packet.value);
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
  Serial.println("START");
  Serial.println("MOVE_FORWARD");
  Serial.println("MOVE_LEFT");
  Serial.println("HOVER");
  Serial.println("LAND");
  Serial.println("T:0.2");
  Serial.println("T:0.0");
}

void loop() {
  readSerialInput();
}
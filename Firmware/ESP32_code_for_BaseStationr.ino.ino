// ESP32 Base Station - LoRa receiver -> SMS via SIM800L
// Requires: LoRa (Sandeep Mistry)
// Wiring: LoRa SPI pins per defines. SIM800L on Serial1 (RX=16, TX=17)

#include <SPI.h>
#include <LoRa.h>

// LoRa pins
#define LORA_SS    18
#define LORA_SCK    5
#define LORA_MOSI  27
#define LORA_MISO  19
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_FREQ  868E6

// SIM800L serial (HardwareSerial 1)
#define SIM_RX_PIN 16 // SIM TX -> ESP32 RX1 (GPIO16)
#define SIM_TX_PIN 17 // SIM RX -> ESP32 TX1 (GPIO17)
HardwareSerial simSerial(1);

// SMS target (change this to the rescuer/authority number including country code)
const char *ALERT_NUMBER = "+91XXXXXXXXXX";

// CRC16-CCITT (same function as user device)
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

// Utility to wait for SIM response (timeout ms) and optionally print to Serial
bool waitForSIMResponse(unsigned long timeout=2000) {
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < timeout) {
    while (simSerial.available()) {
      char c = simSerial.read();
      resp += c;
    }
    if (resp.length()) {
      Serial.print("[SIM] "); Serial.println(resp);
      return true;
    }
  }
  Serial.println("[SIM] no response");
  return false;
}

// Send a single AT command and optionally wait for some response time
void sendAT(const char *cmd, unsigned long wait_ms=500) {
  Serial.print(">> "); Serial.println(cmd);
  simSerial.print(cmd);
  simSerial.print("\r\n");
  delay(wait_ms);
  // read whatever is available and print
  unsigned long t0 = millis();
  while (millis() - t0 < 200) {
    if (simSerial.available()) {
      Serial.write(simSerial.read());
    }
  }
  Serial.println();
}

// Send SMS in plain text
bool sendSMS(const char *number, const String &message) {
  Serial.println("Sending SMS...");
  // Set SMS text mode
  sendAT("AT+CMGF=1", 500);
  delay(200);

  // Set recipient
  simSerial.print("AT+CMGS=\"");
  simSerial.print(number);
  simSerial.print("\"\r\n");
  delay(200);

  // Wait for '>' prompt
  unsigned long start = millis();
  bool prompt = false;
  String buf = "";
  while (millis() - start < 3000) {
    while (simSerial.available()) {
      char c = simSerial.read();
      buf += c;
      if (c == '>') { prompt = true; break; }
    }
    if (prompt) break;
  }
  Serial.print("[SIM prompt?] "); Serial.println(prompt ? "yes" : "no");
  if (!prompt) {
    // fallback: print buffer and return failure
    Serial.print("[SIM buf] "); Serial.println(buf);
    return false;
  }

  // Send message text and ctrl+z
  simSerial.print(message);
  simSerial.write(0x1A); // Ctrl-Z to send
  // Wait for OK or +CMGS:
  unsigned long t0 = millis();
  String resp = "";
  while (millis() - t0 < 10000) {
    while (simSerial.available()) {
      char c = simSerial.read();
      resp += c;
    }
    if (resp.indexOf("OK") >= 0 || resp.indexOf("+CMGS:") >= 0) {
      Serial.print("[SIM] Sent OK: "); Serial.println(resp);
      return true;
    }
  }
  Serial.print("[SIM] Send failed, resp: "); Serial.println(resp);
  return false;
}

// Parse incoming LoRa packet bytes (expects len==20)
bool parsePacket(const uint8_t *buf, size_t len, String &outText) {
  if (len < 20) return false;
  // Validate CRC
  uint16_t recvCrc = ((uint16_t)buf[17] << 8) | buf[18];
  uint16_t calc = crc16_ccitt(buf, 17);
  if (recvCrc != calc) {
    Serial.printf("CRC mismatch: recv=0x%04X calc=0x%04X\n", recvCrc, calc);
    return false;
  }
  // DeviceID
  uint32_t deviceId = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3]);
  uint8_t msgType = buf[4];

  // lat lon
  int32_t lat_i = ((int32_t)buf[5] << 24) | ((int32_t)buf[6] << 16) | ((int32_t)buf[7] << 8) | ((int32_t)buf[8]);
  int32_t lon_i = ((int32_t)buf[9] << 24) | ((int32_t)buf[10] << 16) | ((int32_t)buf[11] << 8) | ((int32_t)buf[12]);
  double lat = lat_i / 1e7;
  double lon = lon_i / 1e7;

  // altitude
  int16_t alt = ((int16_t)buf[13] << 8) | buf[14];

  uint8_t batt = buf[15];
  uint8_t seq = buf[16];

  // Build a readable message
  outText = "";
  outText += "SOS Alert!\n";
  outText += "Device: 0x"; outText += String(deviceId, HEX); outText += "\n";
  outText += "Type: "; outText += String(msgType); outText += "\n";
  outText += "Lat: "; outText += String(lat, 7); outText += "\n";
  outText += "Lon: "; outText += String(lon, 7); outText += "\n";
  outText += "Alt: "; outText += String(alt); outText += " m\n";
  outText += "Battery: "; outText += String(batt); outText += "%\n";
  outText += "Seq: "; outText += String(seq);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32 Base Station starting...");

  // Init SIM serial
  simSerial.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  delay(300);
  // Basic SIM init
  sendAT("AT"); delay(200);
  sendAT("ATE0"); // echo off
  sendAT("AT+CPIN?"); // check SIM
  sendAT("AT+CREG?"); // network reg
  sendAT("AT+CSQ"); // signal quality

  // Init LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin((long)LORA_FREQ)) {
    Serial.println("LoRa init failed. Check wiring.");
    while (1) { delay(1000); }
  }
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setTxPower(14);

  Serial.println("Setup complete. Listening for LoRa packets...");
}

void loop() {
  // Non-blocking receive
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.printf("Received packet size: %d\n", packetSize);
    uint8_t buf[64];
    int idx = 0;
    while (LoRa.available() && idx < sizeof(buf)) {
      buf[idx++] = LoRa.read();
    }
    Serial.printf("Read %d bytes from LoRa\n", idx);
    String smsText;
    if (parsePacket(buf, idx, smsText)) {
      Serial.println("Packet parsed OK. Message:");
      Serial.println(smsText);

      // Send SMS (one-line compact)
      String shortMsg = "";
      // create compact single-line SMS for brevity
      // Format: SOS Device:0xID Lat,Lng Alt m Batt% Seq
      // Use fixed decimal places
      int32_t deviceId = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3]);
      double lat = (((int32_t)buf[5] << 24) | ((int32_t)buf[6] << 16) | ((int32_t)buf[7] << 8) | ((int32_t)buf[8])) / 1e7;
      double lon = (((int32_t)buf[9] << 24) | ((int32_t)buf[10] << 16) | ((int32_t)buf[11] << 8) | ((int32_t)buf[12])) / 1e7;
      int16_t alt = ((int16_t)buf[13] << 8) | buf[14];
      uint8_t batt = buf[15];
      uint8_t seq = buf[16];

      shortMsg += "SOS ";
      shortMsg += "Dev:0x"; shortMsg += String(deviceId, HEX);
      shortMsg += " Lat:"; shortMsg += String(lat, 5);
      shortMsg += " Lon:"; shortMsg += String(lon, 5);
      shortMsg += " Alt:"; shortMsg += String(alt);
      shortMsg += "m Bat:"; shortMsg += String(batt); shortMsg += "%";
      shortMsg += " Seq:"; shortMsg += String(seq);

      Serial.print("SMS to send: ");
      Serial.println(shortMsg);

      bool ok = sendSMS(ALERT_NUMBER, shortMsg);
      if (ok) Serial.println("SMS sent successfully.");
      else Serial.println("SMS failed to send.");
    } else {
      Serial.println("Packet parse/CRC failed; ignoring.");
    }
  }

  delay(50);
}

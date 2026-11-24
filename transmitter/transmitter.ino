/*
 * transmitter.ino
 * -------------------------------------------------
 * Description: Reads distance and signal strength from TF02-Pro LiDAR,
 *              and transmits the data to the receiver ESP32 via ESP-NOW.
 *
 * Pin Connections:
 *   TF02-Pro TX  →  ESP32 GPIO 16 (RX2)
 *   TF02-Pro RX  →  ESP32 GPIO 17 (TX2)
 *   TF02-Pro VCC →  5V
 *   TF02-Pro GND →  GND
 *
 * Receiver ESP32 MAC address must be configured in RECEIVER_MAC below.
 * -------------------------------------------------
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ── MAC Address of Receiver ESP32 ──────────────────────
// Replace this with the MAC address printed by receiver.ino
uint8_t RECEIVER_MAC[6] = {0xE0, 0x8C, 0xFE, 0x32, 0x02, 0xE4};

// ── LiDAR Serial Port ────────────────────────────────
#define LIDAR_RX    16
#define LIDAR_TX    17
#define LIDAR_BAUD  115200
HardwareSerial lidarSerial(2);

// ── Sensor Data Packet structure ─────────────────────
typedef struct {
    uint16_t distance_cm;
    uint16_t strength;
    uint8_t  valid;
} SensorPacket;

SensorPacket packet;

// ── LiDAR State Machine ──────────────────────────────
// TF02-Pro frame: [0x59][0x59][DistL][DistH][StrL][StrH][Rsv][Rsv][Checksum]
//                   0     1     2      3      4     5     6    7     8
enum LidarState { WAIT_H1, WAIT_H2, READ_DATA };
LidarState lidarState = WAIT_H1;
uint8_t    lidarBuf[7];   // 7 bytes after the header
uint8_t    lidarIdx = 0;

// Variables set when a new valid frame is parsed
bool       frameReady    = false;
uint16_t   frameDist     = 0;
uint16_t   frameStrength = 0;

// ── ESP-NOW Transmission Callback ─────────────────────
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[ESPNOW] Transmission FAILED");
    }
}

// ── Non-blocking LiDAR Parser ──────────────────────────
// Consumes all available bytes in the serial buffer without blocking.
// Sets frameReady = true when a full valid frame is parsed.
void parseLiDAR() {
    while (lidarSerial.available()) {
        uint8_t b = lidarSerial.read();

        switch (lidarState) {
            case WAIT_H1:
                if (b == 0x59) lidarState = WAIT_H2;
                break;

            case WAIT_H2:
                if (b == 0x59) {
                    lidarIdx  = 0;
                    lidarState = READ_DATA;
                } else {
                    // Header mismatch, reset
                    lidarState = (b == 0x59) ? WAIT_H2 : WAIT_H1;
                }
                break;

            case READ_DATA:
                lidarBuf[lidarIdx++] = b;
                if (lidarIdx == 7) {
                    // Verify checksum
                    uint8_t cs = 0x59 + 0x59;
                    for (int i = 0; i < 6; i++) cs += lidarBuf[i];

                    if (cs == lidarBuf[6]) {
                        frameDist     = lidarBuf[0] | (lidarBuf[1] << 8);
                        frameStrength = lidarBuf[2] | (lidarBuf[3] << 8);
                        frameReady    = true;
                    }
                    // If checksum is invalid, silently discard frame and wait for next
                    lidarState = WAIT_H1;
                    lidarIdx   = 0;
                }
                break;
        }
    }
}

// ── Transmission Timer ───────────────────────────────
uint32_t lastSendMs = 0;
#define SEND_INTERVAL_MS 50   // 20 Hz transmission rate (every 50ms)

void setup() {
    Serial.begin(115200);
    Serial.println("[TRANSMITTER] Starting...");

    lidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, LIDAR_TX);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] Init ERROR!");
        ESP.restart();
    }
    esp_now_register_send_cb(onSent);

    // Register receiver peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESPNOW] Peer could not be added!");
        ESP.restart();
    }

    Serial.println("[TRANSMITTER] Ready.");
}

void loop() {
    // Process all serial bytes in buffer - NON-BLOCKING
    parseLiDAR();

    // Update payload when a new LiDAR frame is available
    if (frameReady) {
        packet.distance_cm = frameDist;
        packet.strength    = frameStrength;
        packet.valid       = 1;
        frameReady         = false;
    }

    // Send packet every 50 ms
    uint32_t now = millis();
    if (now - lastSendMs >= SEND_INTERVAL_MS) {
        lastSendMs = now;

        esp_now_send(RECEIVER_MAC, (uint8_t *)&packet, sizeof(packet));

        if (packet.valid) {
            Serial.printf("[LiDAR] %d cm | Strength: %d\n",
                          packet.distance_cm, packet.strength);
        } else {
            Serial.println("[LiDAR] Invalid");
        }

        // Reset validity until the next new frame arrives
        packet.valid = 0;
    }
}

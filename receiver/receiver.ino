/*
 * receiver.ino
 * -------------------------------------------------
 * Description: Receives LiDAR data from the transmitter ESP32 via ESP-NOW,
 *              reads speed/satellite count from NEO-6M GPS, and transmits
 *              this data to the PC via USB Serial in CSV format every 100 ms.
 *
 * CSV format:
 *   speed_kmph,satellites,distance_cm,strength,lidar_valid
 *   Example: 45.3,7,823,180,1
 *
 * Pin Connections:
 *   NEO-6M TX  →  ESP32 GPIO 16 (RX2)
 *   NEO-6M RX  →  ESP32 GPIO 17 (TX2)
 *   NEO-6M VCC →  3.3V
 *   NEO-6M GND →  GND
 *
 * Libraries: TinyGPSPlus
 * -------------------------------------------------
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>

// ── GPS Serial Pins ───────────────────────────────────
#define GPS_RX    16
#define GPS_TX    17
#define GPS_BAUD  9600      // NEO-6M default baud rate

HardwareSerial gpsSerial(2);   // UART2
TinyGPSPlus    gps;

// ── Received Data Packet (matches transmitter struct) ──
typedef struct {
    uint16_t distance_cm;
    uint16_t strength;
    uint8_t  valid;
} SensorPacket;

// Volatile variables for incoming LiDAR data (received via callback)
volatile SensorPacket latestLidar = {0, 0, 0};
volatile bool newLidarData = false;

// ── ESP-NOW Receive Callback ───────────────────────────
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len == sizeof(SensorPacket)) {
        memcpy((void *)&latestLidar, data, sizeof(SensorPacket));
        newLidarData = true;
    }
}

// ── UBX Command Helper ─────────────────────────────────
void sendUBX(HardwareSerial &ser, const uint8_t *cmd, uint8_t len) {
    ser.write(cmd, len);
    delay(120);
}

// ── NEO-6M GPS Configuration ───────────────────────────
// Enable only GGA + RMC, disable others for clean parsing
void configureGPS() {
    // Disable GSA
    static const uint8_t disableGSA[] = {
        0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,
        0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x31
    };
    // Disable GSV
    static const uint8_t disableGSV[] = {
        0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,
        0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38
    };
    // Disable GLL
    static const uint8_t disableGLL[] = {
        0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A
    };
    // Disable VTG
    static const uint8_t disableVTG[] = {
        0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,
        0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46
    };
    // Enable GGA
    static const uint8_t enableGGA[] = {
        0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,
        0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x28
    };
    // Enable RMC (Speed data comes from here)
    static const uint8_t enableRMC[] = {
        0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,
        0x00,0x01,0x00,0x00,0x00,0x00,0x04,0x40
    };

    sendUBX(gpsSerial, disableGSA, sizeof(disableGSA));
    sendUBX(gpsSerial, disableGSV, sizeof(disableGSV));
    sendUBX(gpsSerial, disableGLL, sizeof(disableGLL));
    sendUBX(gpsSerial, disableVTG, sizeof(disableVTG));
    sendUBX(gpsSerial, enableGGA,  sizeof(enableGGA));
    sendUBX(gpsSerial, enableRMC,  sizeof(enableRMC));
    Serial.println("[GPS] Configuration completed.");
}

void setup() {
    // USB Serial communication with PC
    Serial.begin(115200);
    Serial.println("[RECEIVER] Starting...");

    // Print MAC Address (needs to be copied to transmitter.ino)
    WiFi.mode(WIFI_STA);
    delay(300);
    Serial.print("[INFO] MAC Address of this device: ");
    Serial.println(WiFi.macAddress());

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] Init ERROR! Restarting...");
        ESP.restart();
    }
    esp_now_register_recv_cb(onReceive);
    Serial.println("[ESPNOW] Ready, waiting for incoming data...");

    // Initialize and configure GPS
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(500);
    configureGPS();

    Serial.println("[RECEIVER] System ready.");
    Serial.println("FORMAT: speed_kmph,satellites,distance_cm,strength,lidar_valid");
}

// ── Variables for storing last valid GPS data ──────────
float    lastSpeed = 0.0f;
uint32_t lastSats  = 0;
uint32_t lastSendMs = 0;

void loop() {
    // Read and parse GPS serial stream
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    // Update variables if valid GPS fix is available
    if (gps.speed.isValid())      lastSpeed = gps.speed.kmph();
    if (gps.satellites.isValid()) lastSats  = gps.satellites.value();

    // Send CSV row to PC every 100 ms
    uint32_t now = millis();
    if (now - lastSendMs >= 100) {
        lastSendMs = now;

        // Safely copy volatile LiDAR packet
        SensorPacket pkt;
        noInterrupts();
        memcpy(&pkt, (const void *)&latestLidar, sizeof(SensorPacket));
        interrupts();

        // CSV line output
        Serial.printf("%.1f,%lu,%u,%u,%u\n",
                      lastSpeed,
                      lastSats,
                      pkt.distance_cm,
                      pkt.strength,
                      (unsigned)pkt.valid);
    }
}

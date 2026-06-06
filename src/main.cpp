#include <Arduino.h>

#include <esp_mac.h>

// Configuration options
#define GET_MAC                                                                \
  false // Set to true to print MAC address on startup, false otherwise

// ESP-NOW Robot MAC address lookup table (index matches robot_id)
const uint8_t ROBOT_MACS[][6] = {
  {0x98, 0x3D, 0xAE, 0xB4, 0xB6, 0x44}  // Robot 0
};


// Struct representing the packet payload (8 bytes total)
struct __attribute__((__packed__)) RobotPacket {
  uint8_t header;
  uint8_t length;
  uint8_t robot_id;
  int16_t wheel_left;
  int16_t wheel_right;
  uint8_t crc;
};

// Compute CRC-8 Dallas/Maxim — matches the Python/sender implementation.
uint8_t crc8_dallas(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    uint8_t inbyte = data[i];
    for (int j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) {
        crc ^= 0x8C;
      }
      inbyte >>= 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB)
  }
  Serial.println(
      "ESP32 Robot Bridge Initialized. Ready to receive commands...");
}

void loop() {
#if GET_MAC
  uint8_t mac[6];
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
    Serial.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0],
                  mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
#endif

  static uint8_t buffer[8];
  static size_t bytes_read = 0;

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    // State machine to parse the 8-byte packet
    if (bytes_read == 0) {
      if (b == 0xAA) {
        buffer[0] = b;
        bytes_read = 1;
      }
    } else if (bytes_read == 1) {
      if (b == 0x08) { // The expected packet length is 8
        buffer[1] = b;
        bytes_read = 2;
      } else {
        // Invalid length, reset or restart sync if this byte is a header
        if (b == 0xAA) {
          buffer[0] = 0xAA;
          bytes_read = 1;
        } else {
          bytes_read = 0;
        }
      }
    } else if (bytes_read < 8) {
      buffer[bytes_read] = b;
      bytes_read++;

      // Once all 8 bytes are received, process the packet
      if (bytes_read == 8) {
        uint8_t calculated_crc = crc8_dallas(buffer, 7);
        uint8_t received_crc = buffer[7];

        if (calculated_crc == received_crc) {
          RobotPacket *packet = (RobotPacket *)buffer;

          // Reconstruct floating-point values from scaled short values
          float wl = (float)packet->wheel_left / 100.0f;
          float wr = (float)packet->wheel_right / 100.0f;

          Serial.printf("PACKET RECEIVED - ID: %d | Left Wheel: %.2f | Right "
                        "Wheel: %.2f | CRC: OK (0x%02X)\n",
                        packet->robot_id, wl, wr, received_crc);
        } else {
          Serial.printf("PACKET ERROR - CRC mismatch: calculated 0x%02X, "
                        "received 0x%02X\n",
                        calculated_crc, received_crc);
        }

        // Reset state machine for next packet
        bytes_read = 0;
      }
    }
  }
}
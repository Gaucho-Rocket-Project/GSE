#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR VEHICLE ESP32 MAC ADDRESS
// Example: 24:6F:28:1A:2B:3C becomes {0x24, 0x6F, 0x28, 0x1A, 0x2B, 0x3C}
uint8_t broadcastAddress[] = {0xA4, 0xF0, 0x0F, 0x61, 0x84, 0x3C}; 

typedef struct struct_message {
  uint8_t target; // 0 = Servo0, 1 = Servo1, 2 = Both
  uint8_t state;  // 0 = OFF, 1 = ON
} struct_message;

struct_message commandToSend;
esp_now_peer_info_t peerInfo;

// FIXED: Updated first parameter to match the new ESP32 v3.1+ library requirements
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("ACK: Command Delivered");
  } else {
    Serial.println("ERR: Delivery Failed");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("Ground Station Ready.");
}

void loop() {
  // Listen for commands from the Python script
  // Protocol: 2 characters. 
  // Char 1 (Target): '0', '1', or 'B' (Both)
  // Char 2 (State):  '0' (Off) or '1' (On)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 2) {
      char t = input.charAt(0);
      char s = input.charAt(1);

      // Parse Target
      if (t == '0') commandToSend.target = 0;
      else if (t == '1') commandToSend.target = 1;
      else if (t == 'B') commandToSend.target = 2;
      else return; // Invalid

      // Parse State
      if (s == '0') commandToSend.state = 0;
      else if (s == '1') commandToSend.state = 1;
      else return; // Invalid

      // Send message via ESP-NOW
      esp_now_send(broadcastAddress, (uint8_t *) &commandToSend, sizeof(commandToSend));
    }
  }
}
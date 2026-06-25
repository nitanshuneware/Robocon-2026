
// no : 2

#include <WiFi.h>
#include <esp_now.h>

// Structure
typedef struct struct_message {
  char command[20];
} struct_message;

struct_message dataToSend;

// 🔴 PUT ESP #3 MAC ADDRESS HERE
uint8_t receiverMAC[] = {0xD4, 0xE9, 0xF4, 0xE2, 0x10, 0x88};
//  D4:E9:F4:E2:10:88
void setup() {
  Serial.begin(250000);  // From ESP #1

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);
}

void loop() {
  if (Serial.available()) {
    String v = Serial.readStringUntil('\n');
    v.trim();

    v.toCharArray(dataToSend.command, 20);

    esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

    Serial.print("Forwarded: ");
    Serial.println(v);
  }
}
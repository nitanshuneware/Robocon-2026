#include <WiFi.h>
#include <esp_now.h>
#include <CytronMotorDriver.h>

// Motors
CytronMD kfb(PWM_DIR, 19, 18);
CytronMD kfr(PWM_DIR, 17, 16);
CytronMD kfl(PWM_DIR, 4, 2);

int kfs = 27;
int kfs_lift = 255;
int kfs_rotate = 255;
int kfs_hand = 255;

String v = "";
String curr_kfs = "", pre_kfs = "";
int hand = 0;

// Structure
typedef struct struct_message {
  char command[20];
} struct_message;

struct_message incomingData;

// ✅ FIXED CALLBACK
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataBytes, int len) {

  memcpy(&incomingData, incomingDataBytes, min(len, (int)sizeof(incomingData)));

  v = String(incomingData.command);
  v.trim();

  Serial.print("Received: ");
  Serial.println(v);
}

void setup() {
  Serial.begin(250000);

  pinMode(kfs, OUTPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {

  if (v == "KF_up") {
    kfl.setSpeed(kfs_lift);
  }
  else if (v == "KF_dn") {
    kfl.setSpeed(-kfs_lift);
  }
  else if (v == "KFH_F") {
    kfr.setSpeed(-kfs_rotate);
  }
  else if (v == "KFH_B") {
    kfr.setSpeed(kfs_rotate);
  }
  else if (v == "KF_C") {
    kfb.setSpeed(-kfs_hand);
  }
  else if (v == "KF_AC") {
    kfb.setSpeed(kfs_hand);
  }
  else if (v == "S") {
    kfl.setSpeed(0);
    kfr.setSpeed(0);
    kfb.setSpeed(0);
  }

  if (v == "KP" || v == "KP1") {
    curr_kfs = v;
    if (curr_kfs == "KP" && pre_kfs == "KP1") hand++;
    pre_kfs = curr_kfs;
  }

  digitalWrite(kfs, (hand % 2 == 1) ? HIGH : LOW);
}
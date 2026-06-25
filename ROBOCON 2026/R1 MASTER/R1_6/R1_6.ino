#include "HardwareSerial.h"
#include <CytronMotorDriver.h>

HardwareSerial espSerial(0);

CytronMD kfl(PWM_DIR, 4,  2);
CytronMD kfr(PWM_DIR, 17, 16);
CytronMD kfb(PWM_DIR, 19, 18);

const int st = 27;   
const int kf = 14; 

const int kfs_lift   = 255;
const int kfs_rotate = 255;
const int kfs_hand   = 255;

int p_1 = 0, p_2 = 0;
String prev_st_cmd = "p1";
String prev_kf_cmd = "p2";

String v = "";

void setup() {
  Serial.begin(115200);
  espSerial.begin(250000);

  pinMode(pin_st, OUTPUT);
  pinMode(pin_kf, OUTPUT);

  digitalWrite(pin_st, LOW);
  digitalWrite(pin_kf, LOW);
}

void loop() {

  if (espSerial.available()) {
    v = espSerial.readStringUntil('\n');
    v.trim();
  }

  if      (v == "stu") { kfr.setSpeed(-kfs_rotate); }   
  else if (v == "std") { kfr.setSpeed( kfs_rotate); }   
  else if (v == "sta") { kfl.setSpeed(-kfs_lift);   }   
  else if (v == "stc") { kfl.setSpeed( kfs_lift);   }
  else if (v == "kff") { kfb.setSpeed(-kfs_hand); }   
  else if (v == "kfb") { kfb.setSpeed( kfs_hand); }     
  else if (v == "kfa") { kfr.setSpeed(-kfs_rotate); }   
  else if (v == "kfc") { kfr.setSpeed( kfs_rotate); }   

  else if (v == "S") {
    kfl.setSpeed(0);
    kfr.setSpeed(0);
    kfb.setSpeed(0);
  }

  if (v == "P1" || v == "p1") {
    if (v == "P1" && prev_st_cmd == "p1") p_1++;
    prev_st_cmd = v;
  }
  digitalWrite(st, (p_1 % 2 == 1) ? HIGH : LOW);

  if (v == "P2" || v == "p2") {
    if (v == "P2" && prev_kf_cmd == "p2") p_2++;
    prev_kf_cmd = v;
  }
  digitalWrite(kf, (p_2 % 2 == 1) ? HIGH : LOW);

  Serial.println(v);
}

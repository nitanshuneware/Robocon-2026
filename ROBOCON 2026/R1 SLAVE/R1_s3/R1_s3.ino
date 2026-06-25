#include "HardwareSerial.h"
#include <CytronMotorDriver.h>

String v = "" , curr_kfs , pre_kfs;
int hand ;
int kfs = 27 ;
int kfs_lift = 255;
int kfs_rotate = 255;
int kfs_hand = 255;

HardwareSerial espSerial(0);

CytronMD kfb(PWM_DIR, 19, 18);
CytronMD kfr(PWM_DIR, 17, 16);
CytronMD kfl(PWM_DIR, 4, 2);

void setup() {
  espSerial.begin(250000); 
  pinMode(kfs,  OUTPUT)  ;
}

void loop() {

  if (espSerial.available()) {
    v = espSerial.readStringUntil('\n');
    v.trim();
  }


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
  if(v == "KP" || v == "KP1" ){
  curr_kfs = v;
  if(curr_kfs == "KP" && pre_kfs == "KP1") hand++ ;
  pre_kfs = curr_kfs ;
  }
  if(hand%2 == 1)  digitalWrite(kfs, HIGH);
  else digitalWrite(kfs, LOW);


}
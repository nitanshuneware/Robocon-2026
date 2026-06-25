#include "HardwareSerial.h"
#include <CytronMotorDriver.h>

HardwareSerial espSerial(0);

CytronMD str(PWM_DIR, 17, 5);
CytronMD kfsf(PWM_DIR, 18, 19);

// CytronMD stp(PWM_DIR, 22, 23);  //staff

const int kf = 32; 
const int st = 33;

const int st_r    = 100 ;
const int kf_fb   = 255;

int p_1 = 0, p_2 = 0 ;
String prev_st ;
String prev_kf ;

String v = "" , prev_v = "";

void setup() {
  espSerial.begin(250000);
	// myservo.attach(servoPin); 
  pinMode(st, OUTPUT);
  pinMode(kf, OUTPUT);
  digitalWrite(st, LOW);
  digitalWrite(kf, LOW);
}

void loop() {

  if (espSerial.available()) {
    v = espSerial.readStringUntil('\n');
    v.trim();
  }
  if (v != "" && v != prev_v) {

   if (v == "sta") { str.setSpeed(st_r);   }   
  else if (v == "stc") { str.setSpeed( -st_r);   }
  else if (v == "kff") { kfsf.setSpeed(kf_fb); }   
  else if (v == "kfb") { kfsf.setSpeed( -kf_fb); }
  // else if (v == "stp") { stp.setSpeed(kf_fb); }   
  // else if (v == "sts") { stp.setSpeed( -kf_fb); }

  else if (v == "S") {
    str.setSpeed(0);
    kfsf.setSpeed(0);
    // stp.setSpeed(0);
  }

  if (v == "P1" || v == "p1") {
    if (v == "P1" && prev_st == "p1") p_1++;
    prev_st = v;
  }
  digitalWrite(st, (p_1 % 2 == 1) ? HIGH : LOW);

  if (v == "P2" || v == "p2") {
    if (v == "P2" && prev_kf == "p2") p_2++;
    prev_kf = v;
  }
  digitalWrite(kf, (p_2 % 2 == 1) ? HIGH : LOW);

  // if (v == "S1" || v == "s1") {
  // if (v == "S1" && prev_s == "s1") s_1++;
  // prev_s = v;
  // }
  //  (s_1 % 2 == 1) ? myservo.write(120) : myservo.write(0);

  prev_v = v;  

  }
  Serial.println(v);
}

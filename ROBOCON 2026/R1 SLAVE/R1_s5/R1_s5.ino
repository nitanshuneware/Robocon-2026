#include "HardwareSerial.h"
#include <CytronMotorDriver.h>
#include <ESP32Servo.h>

HardwareSerial espSerial(0);

Servo myservo;

CytronMD plat(PWM_DIR, 4,  2);
// staff
CytronMD str(PWM_DIR, 17, 16);
CytronMD stud(PWM_DIR, 19, 18);
// kfs
CytronMD kfsf(PWM_DIR, 12, 14);
CytronMD kfsr(PWM_DIR, 27, 26);

const int st = 33;   
const int kf = 32; 
const int servoPin = 13;

const int st_ud   = 190;
const int st_r    = 55 ;
const int kf_fb   = 255;
const int kf_r    = 80;
const int pl      = 255;

int p_1 = 0, p_2 = 0, s_1 = 0;
String prev_st ;
String prev_kf ;
String prev_s ;

String v = "" , prev_v = "";

void setup() {
  espSerial.begin(250000);
	myservo.attach(servoPin); 
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

  if      (v == "stu") { stud.setSpeed(st_ud); }   
  else if (v == "std") { stud.setSpeed( -st_ud); }   
  else if (v == "sta") { str.setSpeed(st_r);   }   
  else if (v == "stc") { str.setSpeed( -st_r);   }
  else if (v == "pu") { plat.setSpeed(-pl); }   
  else if (v == "pd") { plat.setSpeed( pl); }   

  else if (v == "S") {
    stud.setSpeed(0);
    str.setSpeed(0);
    kfsf.setSpeed(0);
    kfsr.setSpeed(0);
    plat.setSpeed(0);

  }
  if (v == "kff") { kfsf.setSpeed(kf_fb); }   
  if (v == "kfb") { kfsf.setSpeed( -kf_fb); }
  if (v == "kfss") { kfsf.setSpeed( 0 ); }     
     
  if (v == "kfa") { kfsr.setSpeed(kf_r); }   
  if (v == "kfc") { kfsr.setSpeed( -kf_r); }   
  if (v == "kfrs") { kfsr.setSpeed( 0 ); }   

  if (v == "pu") { plat.setSpeed(-pl); }   
  if (v == "pd") { plat.setSpeed( pl); }   
  if (v == "ps") { plat.setSpeed( 0 ); }   


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

  if (v == "S1" || v == "s1") {
  if (v == "S1" && prev_s == "s1") s_1++;
  prev_s = v;
  }
   (s_1 % 2 == 1) ? myservo.write(120) : myservo.write(0);

  prev_v = v;  

  }
  Serial.println(v);
}

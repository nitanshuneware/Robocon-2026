#include "HardwareSerial.h"
#include <CytronMotorDriver.h>

String v , curr_kfs , pre_kfs ;

int kfs = 27;
int kfs_lift = 255 ;
int kfs_rotate = 255 ;
int kfs_hand = 255 ;
int hand ;
HardwareSerial espSerial(0);
CytronMD kfb(PWM_DIR, 19, 18) ;
CytronMD kfr(PWM_DIR, 17, 16) ;
CytronMD kfl(PWM_DIR, 4, 2) ;
// CytronMD LA(PWM_DIR, 32, 33) ;

void setup() {
  espSerial.begin(250000) ;
  // Serial.begin(115200);
  pinMode( kfs   , OUTPUT ) ;
}

void loop() {
  if(espSerial.available()){
    v = espSerial.readStringUntil('\n') ; 
    v.trim() ;
  }
  // espSerial.println(v);
  if( v == "KF_up" || v == "KF_dn" || v == "KF_s" || v == "KF_C" || v == "KF_AC" || v == "KFH_F" || v == "KFH_B" || v == "KP" || v == "KP1"  || v == "KF_S"|| v == "KFH_S" ){
    if( v == "KF_up" )      { kfl.setSpeed(kfs_lift) ; espSerial.println(v);}
    else if (v == "KF_dn")  { kfl.setSpeed(-kfs_lift) ;espSerial.println(v);}
    else if( v == "KF_C")   { kfr.setSpeed(kfs_rotate) ;espSerial.println(v);}
    else if( v == "KF_AC")  { kfr.setSpeed(-kfs_rotate) ;espSerial.println(v);}
    else if( v == "KFH_F")   {kfb.setSpeed(kfs_hand) ;espSerial.println(v);}
    else if( v == "KFH_B") {  kfb.setSpeed(-kfs_hand) ;espSerial.println(v);}
    else                    { kfl.setSpeed(0) ; kfr.setSpeed(0) ;kfb.setSpeed(0) ;}

    // if(v == "KP" || v == "KP1" ){
    //   curr_kfs = v;
    //   if(curr_kfs == "KP" && pre_kfs == "KP1") hand++ ;
    //   pre_kfs = curr_kfs ;
    // }
    // if(hand%2 == 1)  digitalWrite(kfs, HIGH);
    // else digitalWrite(kfs, LOW);

  }
  // else{
  //   kfl.setSpeed(0) ;
  //   kfr.setSpeed(0) ;
  //   kfb.setSpeed(0) ;
  //   digitalWrite(kfs, LOW);
  // }
      // espSerial.println(hand);

}
#include "HardwareSerial.h"
#include <CytronMotorDriver.h>
#include <ESP32Servo.h>

Servo esc;

HardwareSerial espSerial(0);

CytronMD m2(PWM_DIR, 14, 27) ;
CytronMD m1(PWM_DIR, 13, 12) ;
CytronMD m4(PWM_DIR, 33, 32) ;
CytronMD m3(PWM_DIR, 26, 25) ;

int servo1 =  ;
int servo2 =  ;

void setup() {

  espSerial.begin(250000) ;

  esc.attach( servo1 );
  esc.attach( servo2 );
}

void loop() {
  if(espSerial.available()){
    var = espSerial.readStringUntil('\n') ; 
    var.trim() ;
  }
//===================================== servo up and down =================
  if( var == "A"){
    	for (int pos = 0; pos <= 100; pos += 1) { 
		esc.write(pos);
		esc.write(pos);    
		delay(15);             
	}
	for (int pos = 100; pos >= 0; pos -= 1) {
		esc.write(pos);
		esc.write(pos);    
		delay(15);             
	}
}
//=============================  end of 1 ====================================

else if (condition) {
statements
}
  }

}

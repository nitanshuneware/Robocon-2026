#include <ps5Controller.h>
#include <CytronMotorDriver.h>
#include "BNO055_support.h"		
#include <Wire.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

struct bno055_t myBNO;
struct bno055_euler myEulerData; 
unsigned long lastTime = 0;

CytronMD m1(PWM_DIR, 25, 26) ;
CytronMD m2(PWM_DIR, 27, 14) ;
CytronMD m3(PWM_DIR, 12, 13) ;
CytronMD m4(PWM_DIR, 32, 33) ;

int start;
int curr_srt , pre_srt ;
int curr_r , pre_r ;
int curr_l , pre_l ;
int spd = 150 ;

void setup() {

  Wire.begin();
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(1);


  Serial.begin(115200);
 pinMode( 2, OUTPUT);
  // ps5.begin("A0:FA:9C:0D:D5:BE");
  ps5.begin("7C:66:EF:3C:EF:D5");
  // ps5.begin("D0:BC:C1:A3:40:83");

  xTaskCreatePinnedToCore( Maincode, "Task1", 10000, NULL, 0, &Task1, 0); 
  xTaskCreatePinnedToCore(Orientation, "Task2", 10000, NULL, 1, &Task2,  1);        


}

void loop() {
//==================== LIGHTNING-95=======================//
  }


void Maincode( void * parameter) {
  while(1) {
  if (ps5.isConnected() == true) {
    digitalWrite(2,HIGH);

    if(ps5.PSButton()){
      abort();
    }
    else{

      curr_srt = ps5.Share();
      if(curr_srt != pre_srt && curr_srt == 1) start++ ;
      pre_srt = curr_srt ;

      if( start%2 == 1){
        curr_r = ps5.R1() ;
        curr_l = ps5.L1() ;

        if(curr_r != pre_r && curr_r == 1)   spd += 5 ; // chessis speed increment
        if(curr_l != pre_l && curr_l == 1)   spd -= 5 ; // chessis speed decrement
        pre_r = curr_r ;
        pre_l = curr_l ;

        spd = constrain(spd , 0 , 255 );
        Serial.print(spd);
        Serial.print("\t");
        // if ( ps5.R2Value() > 30 ) spd = map( ps5.R2Value() , 0 , 255 , 0 , 150 ) ;
        // else if ( ps5.L2Value() > 30 ) spd = map( ps5.L2Value() , 0 , 255 , 0 , 150 ) ;
        if (ps5.Right()) {right();}
        else if (ps5.Left()){ left();}
        else if (ps5.Up()){forward();}
        else if (ps5.Down()) {back();}
        else stop();
        Serial.println();
                //     if (ps5.Triangle()) Serial.println("Triangle Button");

                // if (ps5.UpRight()) Serial.println("Up Right");
                // if (ps5.DownRight()) Serial.println("Down Right");
                // if (ps5.UpLeft()) Serial.println("Up Left");
                // if (ps5.DownLeft()) Serial.println("Down Left");

                // if (ps5.L1()) Serial.println("L1 Button");
                // if (ps5.R1()) Serial.println("R1 Button");

                // if (ps5.Share()) Serial.println("Share Button");
                // if (ps5.Options()) Serial.println("Options Button");
                // if (ps5.L3()) Serial.println("L3 Button");
                // if (ps5.R3()) Serial.println("R3 Button");

                // if (ps5.PSButton()) Serial.println("PS Button");
                // if (ps5.Touchpad()) Serial.println("Touch Pad Button");

                // if (ps5.L2()) {
                //   Serial.printf("L2 button at %d\n", ps5.L2Value());
                // }
                // if (ps5.R2()) {
                //   Serial.printf("R2 button at %d\n", ps5.R2Value());
                // }

                // if (ps5.LStickX()) {
                //   Serial.printf("Left Stick x at %d\n", ps5.LStickX());
                // }
                // if (ps5.LStickY()) {
                //   Serial.printf("Left Stick y at %d\n", ps5.LStickY());
                // }
                // if (ps5.RStickX()) {
                //   Serial.printf("Right Stick x at %d\n", ps5.RStickX());
                // }
                // if (ps5.RStickY()) {
                //   Serial.printf("Right Stick y at %d\n", ps5.RStickY());
                // }


                // Serial.println();
                // // This delay is to make the output more human readable
                // // Remove it when you're not trying to see the output
                // delay(300);


        }
      }
    }
  }
}

void Orientation( void * pvParameters ){
  while(1){
  if ((millis() - lastTime) >= 100)
  {
    lastTime = millis();
    bno055_read_euler_hrp(&myEulerData);			

    Serial.print("Heading(Yaw): ");				
    Serial.println(float(myEulerData.h) / 16.00);	
  }	
}

void forward(){

  m1.setSpeed(spd);      m2.setSpeed(spd);
  m3.setSpeed(spd);      m4.setSpeed(spd);
  Serial.print("Forward \t");

}
void back(){

  m1.setSpeed(-spd);      m2.setSpeed(-spd);
  m3.setSpeed(-spd);      m4.setSpeed(-spd);
  Serial.print("Back\t");

}
void left(){

  m1.setSpeed(spd);      m2.setSpeed(spd);
  m3.setSpeed(spd);      m4.setSpeed(spd);
  Serial.print("Left \t");

}
void right(){

  m1.setSpeed(spd);      m2.setSpeed(spd);
  m3.setSpeed(spd);      m4.setSpeed(spd);
  Serial.print("Right \t");
}
void clk(){

  m1.setSpeed(spd);      m2.setSpeed(spd);
  m3.setSpeed(spd);      m4.setSpeed(spd);
  Serial.print("Clock \t");
}
void aclk(){

  m1.setSpeed(spd);      m2.setSpeed(spd);
  m3.setSpeed(spd);      m4.setSpeed(spd);
  Serial.print("Anti_clock \t");
}
void stop(){

  m1.setSpeed(0);      m2.setSpeed(0);
  m3.setSpeed(0);      m4.setSpeed(0);
  Serial.print("Stop \t");


}



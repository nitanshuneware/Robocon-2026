#include <ps5Controller.h>
#include <CytronMotorDriver.h>
#include "HardwareSerial.h"
#include <ESP32Servo.h>
#include "BNO055_support.h"		
#include <Wire.h>
#include <ESP32Servo.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

Servo esc;

// ================ Chassis ===============
CytronMD m2(PWM_DIR, 14, 27);
CytronMD m1(PWM_DIR, 13, 12);
CytronMD m4(PWM_DIR, 33, 32);
CytronMD m3(PWM_DIR, 26, 25);

HardwareSerial espSerial(0);

// ============== PLATFORM ===============
int bldc = 4;

struct bno055_t myBNO;
struct bno055_euler myEulerData; 
unsigned long lastTime = 0;

float c_kp = 2, c_ki = 0.0, c_kd = 5                                                                                                                                                         ;
float set = 0, o = 0, prev = 0, inte = 0;
float error = 0, deri = 0;
float input = 0;

int forward();
int back();
int right();
int left();
int clk();
int aclk();
int pid();

int start;
int curr_srt , pre_srt;
int curr_r , pre_r;
int curr_l , pre_l;
int mode , curr_mode , pre_mode;

int spd = 50;
int x , y;
int pid_o;
int input_angle , n_angle , c_set;
int perr;

int bldc_s;
int bldc_su = 1560;
int bldc_sd = 1480;

int pbi , cbi;
int pbd , cbd;

int ch_relay = 19;

void setup() {

  xTaskCreatePinnedToCore(Maincode, "Task1", 10000, NULL, 0, &Task1, 0); 
  xTaskCreatePinnedToCore(Orientation, "Task2", 10000, NULL, 1, &Task2, 1);        
 
  // Serial.begin(115200);
  espSerial.begin(250000);

  pinMode(2 , OUTPUT);
  pinMode(ch_relay , OUTPUT);

  ps5.begin("A0:FA:9C:0D:D5:BE");
  // ps5.begin("7C:66:EF:3C:EF:D5");
  // ps5.begin("D0:BC:C1:A3:40:83");

  Wire.begin();
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  esc.attach(bldc);
  esc.writeMicroseconds(1500);
  delay(300);
}

void loop(){
  // ============================= LIGHTNING 95 ======================
}

void Maincode(void * parameter) {
  while(1) {

    if (ps5.isConnected() == true) {
      digitalWrite(2,HIGH);

      if(ps5.PSButton()){
        abort();
      }
      else{

        curr_srt = ps5.Share();
        if(curr_srt != pre_srt && curr_srt == 1) start++;
        pre_srt = curr_srt;

        if(start%2 == 1){

          digitalWrite(ch_relay , HIGH);

          // ===== MODE BUTTON =====
          curr_mode = ps5.Options();
          if(curr_mode != pre_mode && curr_mode == 1) mode++;
          pre_mode = curr_mode;

          Serial.print("Mode = ");
          Serial.print(mode%2);

          curr_r = ps5.R1();
          curr_l = ps5.L1();
          cbi = ps5.R3();
          cbd = ps5.L3(); 
          if(curr_r != pre_r && curr_r == 1) spd += 5; // CHASSIS INCREMENT
          if(curr_l != pre_l && curr_l == 1) spd -= 5; // CHASSIS DECREMENT
          if(cbi != pbi && cbi == 1) bldc_s += 10;// BLDC INCREMENT
          if(cbd != pbd && cbd == 1) bldc_s -= 5;// BLDC DECREMENT
          pre_r = curr_r;
          pre_l = curr_l;
          pbi = cbi;
          pbd = cbd;

          spd = constrain(spd , 0 , 255);

          Serial.print("\tspd = ");
          Serial.print(spd);
          Serial.print("\tpid_o = ");
          Serial.print(pid_o);
          Serial.print("\tX = ");
          Serial.print(x);
          Serial.print("\tY = ");
          Serial.print(y);
          Serial.print("\t");

          pid_o = pid(c_kp , c_ki , c_kd , n_angle , 0);
          x = constrain(spd + pid_o , 0 , 255);
          y = constrain(spd - pid_o , 0 , 255);

          if(ps5.R2Value() > 30 || ps5.L2Value() > 30 || ps5.Right() || ps5.Left() || ps5.Up() || ps5.Down()){        
            if ( ps5.R2Value() > 30 )      { clk(map(ps5.R2Value(),0,255,0,50)); c_set = input_angle; }
            else if ( ps5.L2Value() > 30 ) { aclk(map(ps5.L2Value(),0,255,0,50)); c_set = input_angle; }
            else if (ps5.Right())          right(x,y);
            else if (ps5.Left())           left(x,y);
            else if (ps5.Up())             forward(x,y);
            else if (ps5.Down())           back(x,y);
          } 
          else stop();

          if (ps5.RStickY() > 50)          espSerial.println("KF_up");
          else if (ps5.RStickY() < -50)    espSerial.println("KF_dn");
          else if (ps5.Cross())            espSerial.println("KF_C");
          else if (ps5.Circle())           espSerial.println("KF_AC");
          else if (ps5.Triangle())         espSerial.println( "KFH_F");
          else if (ps5.Square())           espSerial.println("KFH_B");
          else                             espSerial.println ("S");
          ps5.Touchpad() ? espSerial.println("KP") : espSerial.println("KP1");


        }
        else{
          digitalWrite(ch_relay , LOW);
          stop();
        }
      }
    }
    else{
      stop();
      digitalWrite(2 , LOW);
      Serial.println();

    }
  }
  Serial.println();
}

void Orientation( void * pvParameters ){
  while(1){
  if ((millis() - lastTime) >= 100)
  {
    lastTime = millis();
    bno055_read_euler_hrp(&myEulerData);			

    input_angle = (myEulerData.h) / 16.00 ;
    
    if      (input_angle == 360)                       input_angle = 0 ;
    else if (input_angle > 180)                        input_angle = input_angle - 360 ; 
    if      (input_angle - c_set < -180)               n_angle = input_angle - c_set + 360 ;
    else if (input_angle - c_set > 180 )               n_angle = input_angle - c_set - 360 ;
    else                                               n_angle = input_angle - c_set ;
   }	
  }
}
int pid(double kp , double ki , double kd , int av , int sp){
  int err = av - sp ;
  int prop = err ;
  int integral = err + perr ;
  int deri = err-perr ;
  int rslt = prop*kp + integral*ki + deri*kd ;
  perr = err ;
  return(rslt) ;
}
void forward( int a , int b ){

  m1.setSpeed( - a);      m2.setSpeed( - a);
  m3.setSpeed(-b);      m4.setSpeed(-b);
  Serial.print("Forward \t");

}
void back( int a , int b ){

  m1.setSpeed( b);      m2.setSpeed( b);
  m3.setSpeed(  a);      m4.setSpeed(  a);
  Serial.print("Back\t");

}
void right( int a , int b ){

  m1.setSpeed(b);      m2.setSpeed(  -a);
  m3.setSpeed(a);      m4.setSpeed(  -b);
  Serial.print("Left \t");

}
void left( int a , int b ){

  m1.setSpeed( -a);      m2.setSpeed(b);
  m3.setSpeed( -b);      m4.setSpeed(a);
  Serial.print("Right \t");
}
void clk( int z ){

  m1.setSpeed(z);      m2.setSpeed(z);
  m3.setSpeed(-z);      m4.setSpeed(-z);
  Serial.print("Clock \t");

}
void aclk( int z ){

  m1.setSpeed(-z);      m2.setSpeed(-z);
  m3.setSpeed(z);      m4.setSpeed(z);
  Serial.print("Anti_clock \t");

}
void stop(){

  m1.setSpeed(0);      m2.setSpeed(0);
  m3.setSpeed(0);      m4.setSpeed(0);
  esc.writeMicroseconds(1527) ;
  Serial.print("Stop \t");

  // Motordebug();

}
void Motordebug(){
  m1.setSpeed(0);      m2.setSpeed(0);
  m3.setSpeed(0);      m4.setSpeed(0);
  delay(500);
  m1.setSpeed(100);      m2.setSpeed(0);
  m3.setSpeed(0);      m4.setSpeed(0);
  delay(500);
  m1.setSpeed(-100);      m2.setSpeed(0);
  m3.setSpeed(0);      m4.setSpeed(0);
  delay(500);
  m1.setSpeed(0);      m2.setSpeed(100);
  m3.setSpeed(0);      m4.setSpeed(0);
  delay(500);
  m1.setSpeed(0);      m2.setSpeed(-100);
  m3.setSpeed(0);      m4.setSpeed(0);
  delay(500);
  m1.setSpeed(0);      m2.setSpeed(0);
  m3.setSpeed(100);      m4.setSpeed(0);
  delay(500);
  m1.setSpeed(0);      m2.setSpeed(0);
  m3.setSpeed(-100);      m4.setSpeed(0);
  delay(500);
  m1.setSpeed(0);      m2.setSpeed(0);
  m3.setSpeed(0);      m4.setSpeed(100);
  delay(500);
  m1.setSpeed(0);      m2.setSpeed(0);
  m3.setSpeed(0);      m4.setSpeed(-100);
  delay(500);

}
#include <ps5Controller.h>
#include <CytronMotorDriver.h>
#include "HardwareSerial.h"
#include <ESP32Servo.h>
#include "BNO055_support.h"
#include <Wire.h>


TaskHandle_t Task1;
TaskHandle_t Task2;

const int throttlePin = 4;
const int dirPin      = 2;
const int ch_relay    = 19;
// const int LED_PIN     = 2;  
Servo esc;
Servo dir;

CytronMD m3(PWM_DIR, 13, 12);
CytronMD m4(PWM_DIR, 14, 27);
CytronMD m2(PWM_DIR, 26, 25);
CytronMD m1(PWM_DIR, 33, 32);

HardwareSerial espSerial(0);

struct bno055_t    myBNO;
struct bno055_euler myEulerData;
unsigned long lastTime = 0;

// float c_kp = 2.4, c_ki = 0.0001, c_kd = 5.0;
float curr_vx = 0, curr_vy = 0, curr_wz = 0;

const float accel = 0.08f;  
// float curr_vx = 0, curr_vy = 0, curr_wz = 0;
int prev_wz = 0;
unsigned long brake_timer = 0;
bool braking = false;
volatile bool is_rotating = false;

// for samm speed
float c_kp = 2, c_ki = 0.01, c_kd = 1.2;

volatile int input_angle = 0;
volatile int n_angle     = 0;
volatile int c_set       = 0;

int start    = 0;
int mode     = 0;
int spd      = 70;

int mode1     = 0;
int spd2      = 180;


int pid_o    = 0;
int chassis_x = 0, chassis_y = 0;  

int stp = 1500;
int x = 1500;
int ps;

int curr_srt, pre_srt;
int curr_r,   pre_r;
int curr_l,   pre_l;
int curr_mode, pre_mode;
int cbi, pbi;
int cbd, pbd;


///
int curr_mode1, pre_mode1;
int curr_r1,   pre_r1;
int curr_l1,   pre_l1;


float pid_integral = 0.0;
int   pid_prev_err = 0;
int vx, vy, wz;


int  pid_calc(float kp, float ki, float kd, int av, int sp);
void stopAll(int p);
void Maincode   (void* parameter);
void Orientation(void* pvParameters);
void cmode1();
void cmode2();
// ======================================================
void setup() {
  Serial.begin(115200);
  espSerial.begin(250000);

  // pinMode(LED_PIN,  OUTPUT);
  pinMode(ch_relay, OUTPUT);

  // ps5.begin("A0:FA:9C:0D:D5:BE");
  ps5.begin("7C:66:EF:3C:EF:D5");
  // ps5.begin("D0:BC:C1:A3:40:83");

  Wire.begin();
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  esc.attach(throttlePin, 1000, 2000);
  dir.attach(dirPin, 1000, 2000);
  esc.writeMicroseconds(1000);
  dir.writeMicroseconds(1000);
  delay(300);

  xTaskCreatePinnedToCore(Maincode,    "Task1", 10000, NULL, 1, &Task1, 0);
  xTaskCreatePinnedToCore(Orientation, "Task2", 10000, NULL, 1, &Task2, 1);
}

void loop() {
}

void Maincode(void* parameter) {
  while (1) {

    if (ps5.isConnected()) {
      // digitalWrite(LED_PIN, HIGH);

      if (ps5.PSButton() && ps5.Options()) {
        stopAll(0);
        start = 0;
        vTaskDelay(200 / portTICK_PERIOD_MS);
        continue;
      }

      curr_srt = ps5.Share();
      if (curr_srt && !pre_srt) start++;
      pre_srt = curr_srt;

      if (start % 2 == 1) {
        digitalWrite(ch_relay, HIGH);

        curr_mode = ps5.Options();
        if (curr_mode && !pre_mode) mode++;
        pre_mode = curr_mode;

        /////
        curr_mode1 = ps5.PSButton() && !ps5.Options();
        if (curr_mode1 && !pre_mode1) mode1++;
        pre_mode1 = curr_mode1;


        curr_r = ps5.R1();
        curr_l = ps5.L1();
        if (curr_r && !pre_r) spd += 5;
        if (curr_l && !pre_l) spd -= 5;
        pre_r = curr_r;
        pre_l = curr_l;
        spd = constrain(spd, 0, 255);
        spd2 = constrain(spd2, 0, 255);

        ps = ps5.RStickY();

        pid_o    = pid_calc(c_kp, c_ki, c_kd, n_angle, 0);
        pid_o = constrain(pid_o , -100 , 100);

        // Serial.printf("Mode=%d  spd=%d  pid_o=%d  X=%d  Y=%d\n",  mode % 2, spd, pid_o, chassis_x, chassis_y);

        if(mode1 % 2 == 0){
          cmode1();
        }
        else{
          cmode2();
        }

        int rvx = (int)curr_vx;
        int rvy = (int)curr_vy;
        int rwz = (int)curr_wz;

        m1.setSpeed( rvx - rvy + rwz - pid_o);
        m2.setSpeed(-rvx - rvy + rwz - pid_o);
        m3.setSpeed( rvx - rvy - rwz + pid_o);
        m4.setSpeed(-rvx - rvy - rwz + pid_o);

        // m1.setSpeed( vx - vy + wz - pid_o);      
        // m2.setSpeed(-vx - vy + wz - pid_o);
        // m3.setSpeed( vx - vy - wz + pid_o);      
        // m4.setSpeed(-vx - vy - wz + pid_o);

        if (mode % 2 == 0) {
          if      (ps5.RStickY() >  50) espSerial.println("stu");
          else if (ps5.RStickY() < -50) espSerial.println("std");
          else if (ps5.Triangle())       espSerial.println("sta");
          else if (ps5.Circle())         espSerial.println("stc");
          else if (ps5.LStickY() >  50) espSerial.println("pu");
          else if (ps5.LStickY() < -50) espSerial.println("pd");

          else                           espSerial.println("S");

          ps5.Cross() ? espSerial.println("S1") : espSerial.println("s1");
          ps5.Touchpad() ? espSerial.println("P1") : espSerial.println("p1");
        }
        else {
          if (ps > 30) {
            dir.writeMicroseconds(2000); 
          } else {
            dir.writeMicroseconds(1000); 
          }
          if (abs(ps) > 30) {
            x = map(abs(ps), 30, 127, 1050, 1150);
            esc.writeMicroseconds(x);
          } else {
            esc.writeMicroseconds(1000);
          }

          if      (ps5.Triangle())    espSerial.println("kff");
          else if (ps5.Cross())   espSerial.println("kfb");
          else if (ps5.Circle()) espSerial.println("kfa");
          else if (ps5.Square())   espSerial.println("kfc");
          else if (ps5.LStickY() >  50) espSerial.println("pu");
          else if (ps5.LStickY() < -50) espSerial.println("pd");
          else                     espSerial.println("S");

          ps5.Touchpad() ? espSerial.println("P2") : espSerial.println("p2");
        }
      }
      else {
        digitalWrite(ch_relay, LOW);
        esc.writeMicroseconds(1000);
        dir.writeMicroseconds(1000);
        stopAll(0);
        // espSerial.println("S");
      }

    }
    else {
      stopAll(0);
      esc.writeMicroseconds(1000);
      dir.writeMicroseconds(1000);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void cmode1(){
  if (ps5.R2Value() > 30 || ps5.L2Value() > 30 || ps5.Right()   || ps5.Left() || ps5.Up()      || ps5.Down()) {


      wz = map(ps5.R2Value() - ps5.L2Value(),  -255, 255, -95, 95);
      prev_wz = wz;
      braking = false;
      is_rotating  = true;
      was_rotating = true;
  }
  else {
    if (was_rotating && !braking) {
        braking = true;
        brake_timer = millis();
        if (prev_wz > 0) wz = -25;
        else if (prev_wz < 0) wz = 25;
    }
    if (braking) {
        if (millis() - brake_timer > 70) {
            wz = 0;
            braking = false;
            is_rotating  = false;
            was_rotating = false;
        }
    }
    else {  wz = 0; }

    if (ps5.Right())              vx =  -spd;
    else if (ps5.Left())          vx = spd;
    else                          vx = 0;

    if (ps5.Up())                 vy = -spd;    
    else if (ps5.Down())          vy = spd;
    else                          vy = 0;
  }
  else {
    vx = 0;
    vy = 0;
    wz = 0;
  }
  curr_vx = acceleration(curr_vx, (float)vx, accel) ;
  curr_vy = acceleration(curr_vy, (float)vy, accel) ;
  curr_wz = wz ;

}
void cmode2(){
  curr_r1 = ps5.R1();
  curr_l1 = ps5.L1();
  if (curr_r1 && !pre_r1) spd2 += 5;
  if (curr_l1 && !pre_l1) spd2 -= 5;
  pre_r1 = curr_r1;
  pre_l1 = curr_l1;

  if (ps5.R2Value() > 30 || ps5.L2Value() > 30 || ps5.Right()   || ps5.Left() || ps5.Up()      || ps5.Down()) {


      wz = map(ps5.R2Value() - ps5.L2Value(),  -255, 255, -95, 95);
      prev_wz = wz;
      braking = false;
      is_rotating  = true;
      was_rotating = true;
  }
  else {
    if (was_rotating && !braking) {
        braking = true;
        brake_timer = millis();
        if (prev_wz > 0) wz = -25;
        else if (prev_wz < 0) wz = 25;
    }
    if (braking) {
        if (millis() - brake_timer > 70) {
            wz = 0;
            braking = false;
            is_rotating  = false;
            was_rotating = false;
        }
    }
    else {  wz = 0; }

    if (ps5.Right())              vx =  -spd2;
    else if (ps5.Left())          vx = spd2;
    else                          vx = 0;

    if (ps5.Up())                 vy = -spd2;    
    else if (ps5.Down())          vy = spd2;
    else                          vy = 0;
  }
  else {
    vx = 0;
    vy = 0;
    wz = 0;
  }
  curr_vx = acceleration(curr_vx, (float)vx, accel) ;
  curr_vy = acceleration(curr_vy, (float)vy, accel) ;
  curr_wz = wz;

}

void Orientation(void* pvParameters) {

  static int last_angle = 0;
  static unsigned long stable_timer = 0;

  while (1) {

    if ((millis() - lastTime) >= 20) {

      lastTime = millis();

      bno055_read_euler_hrp(&myEulerData);

      int angle = (int)(myEulerData.h / 16.0f);

      if (angle == 360) angle = 0;
      if (angle > 180)  angle -= 360;

      input_angle = angle;

      // =====================================
      // DURING ROTATION
      // =====================================
      if (is_rotating) {

        c_set = angle;
        n_angle = 0;

        stable_timer = millis();
      }

      // =====================================
      // AFTER ROTATION
      // =====================================
      else {

        int delta = abs(angle - last_angle);

        if (delta > 180)
          delta = 360 - delta;

        // Still physically rotating
        if (delta > 1) {

          c_set = angle;
          n_angle = 0;

          stable_timer = millis();
        }

        // Rotation settled
        else {

          // Wait before enabling PID
          if (millis() - stable_timer > 200) {

            int diff = angle - c_set;

            if (diff < -180)
              n_angle = diff + 360;
            else if (diff > 180)
              n_angle = diff - 360;
            else
              n_angle = diff;
          }
          else {

            n_angle = 0;
            c_set = angle;
          }
        }

        last_angle = angle;
      }
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
int pid_calc(float kp, float ki, float kd, int av, int sp) {
  int   err   = av - sp;
  float prop  = err * kp;
  pid_integral += err * ki;               
  pid_integral  = constrain(pid_integral, -100.0, 100.0); 
  float deriv = (err - pid_prev_err) * kd;
  pid_prev_err = err;
  return (int)(prop + pid_integral + deriv);
}
// int pid_calc(float kp, float ki, float kd, int av, int sp) {

//   // =====================================
//   // ROBOT SPEED
//   // =====================================

//   int botSpeed = max(abs(vx), abs(vy));

//   // 1.0 at low speed
//   // 0.6 at high speed
//   float scale = 1.0 - ((float)botSpeed / 255.0) * 0.4;

//   // Adaptive gains
//   float akp = kp * scale;
//   float akd = kd * scale;

//   // =====================================
//   // PID
//   // =====================================

//   int err = av - sp;

//   float prop = err * akp;

//   pid_integral += err * ki;
//   pid_integral = constrain(pid_integral, -80, 80);

//   float deriv = (err - pid_prev_err) * akd;

//   pid_prev_err = err;

//   int pid = prop + pid_integral + deriv;

//   // =====================================
//   // ADAPTIVE OUTPUT LIMIT
//   // =====================================

//   int maxCorrection = map(botSpeed, 0, 255, 120, 55);

//   pid = constrain(pid, -maxCorrection, maxCorrection);

//   return pid;
// }
float acceleration(float current, float target, float factor) {
  return current + (target - current) * factor;

}
void stopAll( int p){
  m1.setSpeed(p);      m2.setSpeed(p);
  m3.setSpeed(p);      m4.setSpeed(p);
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

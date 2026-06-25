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

float c_kp = 2.4, c_ki = 0.0001, c_kd = 5.0;

volatile int input_angle = 0;
volatile int n_angle     = 0;
volatile int c_set       = 0;

int start    = 0;
int mode     = 0;
int spd      = 70;
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

float pid_integral = 0.0;
int   pid_prev_err = 0;


void forward(int a, int b);
void back   (int a, int b);
void right  (int a, int b);
void left   (int a, int b);
void clk    (int z);
void aclk   (int z);
int  pid_calc(float kp, float ki, float kd, int av, int sp);
void stopAll();
void Maincode   (void* parameter);
void Orientation(void* pvParameters);

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

      if (ps5.PSButton()) {
        stopAll();
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

        curr_r = ps5.R1();
        curr_l = ps5.L1();
        if (curr_r && !pre_r) spd += 5;
        if (curr_l && !pre_l) spd -= 5;
        pre_r = curr_r;
        pre_l = curr_l;
        spd = constrain(spd, 0, 255);

        ps = ps5.RStickY();

        pid_o    = pid_calc(c_kp, c_ki, c_kd, n_angle, 0);
        pid_o = constrain(pid_o , -50 , 50);
        chassis_x = constrain(spd + pid_o, 0, 255);
        chassis_y = constrain(spd - pid_o, 0, 255);

        // Serial.printf("Mode=%d  spd=%d  pid_o=%d  X=%d  Y=%d\n",  mode % 2, spd, pid_o, chassis_x, chassis_y);

        if (ps5.R2Value() > 30 || ps5.L2Value() > 30 || ps5.Right()   || ps5.Left() || ps5.Up()      || ps5.Down()) {

          if      (ps5.R2Value() > 30) { clk (map(ps5.R2Value(), 0, 255, 0, 70)); c_set = input_angle; }
          else if (ps5.L2Value() > 30) { aclk(map(ps5.L2Value(), 0, 255, 0, 70)); c_set = input_angle; }
          else if (ps5.Right())        right  (chassis_x, chassis_y);
          else if (ps5.Left())         left   (chassis_x, chassis_y);
          else if (ps5.Up())           forward(chassis_x, chassis_y);
          else if (ps5.Down())         back   (chassis_x, chassis_y);
        }
        else {
          stopAll();
        }

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
            x = map(abs(ps), 30, 127, 1500, 1600);
            esc.writeMicroseconds(x);
          } else {
            esc.writeMicroseconds(1000);
          }

          if      (ps5.Triangle())    espSerial.println("kff");
          else if (ps5.Circle())   espSerial.println("kfb");
          else if (ps5.Cross()) espSerial.println("kfa");
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
        stopAll();
        // espSerial.println("S");
      }

    }
    else {
      stopAll();
      esc.writeMicroseconds(1000);
      dir.writeMicroseconds(1000);
      // digitalWrite(LED_PIN, LOW);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void Orientation(void* pvParameters) {
  while (1) {
    if ((millis() - lastTime) >= 100) {
      lastTime = millis();
      bno055_read_euler_hrp(&myEulerData);

      int angle = (int)(myEulerData.h / 16.0f);
      if (angle == 360) angle = 0;
      if (angle > 180)  angle -= 360;

      input_angle = angle;

      int diff = angle - c_set;
      if      (diff < -180) n_angle = diff + 360;
      else if (diff >  180) n_angle = diff - 360;
      else                  n_angle = diff;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
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

void forward(int a, int b) {
  m1.setSpeed(-a); m2.setSpeed(-a);
  m3.setSpeed(-b); m4.setSpeed(-b);
  // Serial.print("Forward\t");
}

void back(int a, int b) {
  m1.setSpeed(b); m2.setSpeed(b);
  m3.setSpeed(a); m4.setSpeed(a);
  // Serial.print("Back\t");
}

void right(int a, int b) {
  m1.setSpeed(b);  m2.setSpeed(-a);
  m3.setSpeed(a);  m4.setSpeed(-b);
  // Serial.print("Right\t");   
}

void left(int a, int b) {
  m1.setSpeed(-a); m2.setSpeed(b);
  m3.setSpeed(-b); m4.setSpeed(a);
  // Serial.print("Left\t");   
}

void clk(int z) {
  m1.setSpeed(z);  m2.setSpeed(z);
  m3.setSpeed(-z); m4.setSpeed(-z);
  // Serial.print("clk\t");
}

void aclk(int z) {
  m1.setSpeed(-z); m2.setSpeed(-z);
  m3.setSpeed(z);  m4.setSpeed(z);
  // Serial.print("aclk\t");
}

void stopAll() {
  m1.setSpeed(0); m2.setSpeed(0);
  m3.setSpeed(0); m4.setSpeed(0);
  esc.writeMicroseconds(1000);
  dir.writeMicroseconds(1000);
  // Serial.print("Stop\t");
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

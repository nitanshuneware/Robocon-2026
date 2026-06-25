#include <ps5Controller.h>
#include <CytronMotorDriver.h>
#include "HardwareSerial.h"
#include <ESP32Servo.h>
#include "BNO055_support.h"
#include <Wire.h>
#include <math.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

const int ch_relay = 19;

int throttlePin = 16;
int dirPin = 17;

Servo esc;
Servo dir;

int stp = 1500;
int x = 1500;
int ps;


CytronMD m1(PWM_DIR, 32, 33);  // Front wheel
CytronMD m2(PWM_DIR, 27, 14);  // Right wheel
CytronMD m3(PWM_DIR, 12, 13);  // Left wheel

CytronMD st(PWM_DIR, 25, 26);  //staff

HardwareSerial espSerial(0);

struct bno055_t myBNO;
struct bno055_euler myEulerData;
unsigned long lastTime = 0;

int spd = 100;
int sf = 0;
int vx, vy, wz;
float curr_vx = 0, curr_vy = 0, curr_wz = 0;

const float accel = 0.08;

// pid values
float c_kp = 3.0, c_ki = 0.003, c_kd = 3.0;

int c_set = 0;
int input_angle = 0;
int n_angle = 0;

float pid_integral = 0;
int pid_prev_err = 0;
int pid_o = 0;
int start = 0;
int mode = 0;
int ch_mode = 0;

int curr_srt, pre_srt;
int curr_r, pre_r;
int curr_l, pre_l;
int curr_mode, pre_mode;
int curr_ch_mode, pre_ch_mode;

//////////////////
bool was_rotating = false;
int prev_wz = 0;
unsigned long brake_timer = 0;
bool braking = false;
volatile bool is_rotating = false;

const float ANGLE_M1 = 90.0;   //FRONT
const float ANGLE_M2 = -30.0;  //RIGHT
const float ANGLE_M3 = 210.0;  //LEFT

float cosM1, sinM1;
float cosM2, sinM2;
float cosM3, sinM3;

void Maincode(void* parameter);
void Orientation(void* parameter);

int pid(float kp, float ki, float kd, int av, int sp);
float acceleration(float current, float target, float factor);
void stopAll();
void Motordebug();

void setup() {
  Serial.begin(115200);
  espSerial.begin(250000);
  pinMode(ch_relay, OUTPUT);

  ps5.begin("A0:FA:9C:0D:D5:BE");
  // ps5.begin("7C:66:EF:3C:EF:D5");
  // ps5.begin("D0:BC:C1:A3:40:83");

  Wire.begin();
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  esc.attach(throttlePin, 1000, 2000);
  dir.attach(dirPin, 1000, 2000);
  esc.writeMicroseconds(1000);
  dir.writeMicroseconds(1000);
  delay(300);

  xTaskCreatePinnedToCore(Maincode, "Task1", 10000, NULL, 1, &Task1, 0);
  xTaskCreatePinnedToCore(Orientation, "Task2", 10000, NULL, 1, &Task2, 1);

  cosM1 = cosf(ANGLE_M1 * M_PI / 180.0);
  sinM1 = sinf(ANGLE_M1 * M_PI / 180.0);

  cosM2 = cosf(ANGLE_M2 * M_PI / 180.0);
  sinM2 = sinf(ANGLE_M2 * M_PI / 180.0);

  cosM3 = cosf(ANGLE_M3 * M_PI / 180.0);
  sinM3 = sinf(ANGLE_M3 * M_PI / 180.0);
}

void loop() {}

void Maincode(void* parameter) {
  while (1) {
    if (ps5.isConnected()) {

      if (ps5.PSButton() && ps5.Options()) {
        abort();
        stopAll();
        vTaskDelay(200 / portTICK_PERIOD_MS);
        continue;
      }
      curr_srt = ps5.Share();
      if (curr_srt && !pre_srt) { start++; }
      pre_srt = curr_srt;

      if (start % 2 == 1) {
        digitalWrite(ch_relay, HIGH);

        curr_mode = ps5.Options() && !ps5.PSButton();
        if (curr_mode && !pre_mode) mode++;
        pre_mode = curr_mode;
        curr_r = ps5.R1();
        curr_l = ps5.L1();
        if (curr_r && !pre_r) spd += 5;
        if (curr_l && !pre_l) spd -= 5;
        pre_r = curr_r;
        pre_l = curr_l;

        curr_ch_mode = ps5.PSButton() && !ps5.Options();
        if (curr_ch_mode && !pre_ch_mode) ch_mode++;
        pre_ch_mode = curr_ch_mode;

        (ch_mode % 2 == 0) ? sf = 1 : sf = -1;

        spd = constrain(spd, -255, 255);

        ps = ps5.RStickY();
        if (ps5.R2Value() > 30 || ps5.L2Value() > 30 || ps5.Right() || ps5.Left() || ps5.Up() || ps5.Down()) {

          if (ps5.R2Value() > 30 || ps5.L2Value() > 30) {

            wz = map(ps5.R2Value() - ps5.L2Value(), -255, 255, -spd, spd);
            prev_wz = wz;
            braking = false;
            is_rotating = true;
            was_rotating = true;
          } else {

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
                is_rotating = false;
                was_rotating = false;
              }
            } else {
              wz = 0;
            }
          }

          if (ps5.Up()) vx = -spd * sf;
          else if (ps5.Down()) vx = spd * sf;
          else vx = 0;
          if (ps5.Right()) vy = -spd * sf;
          else if (ps5.Left()) vy = spd * sf;
          else vy = 0;

        } else {
          vx = 0;
          vy = 0;
          wz = 0;
        }
        curr_vx = acceleration(curr_vx, (float)vx, accel);
        curr_vy = acceleration(curr_vy, (float)vy, accel);
        curr_wz = wz;

        int rvx = (int)curr_vx;
        int rvy = (int)curr_vy;
        int rwz = (int)curr_wz;

        pid_o = pid(c_kp, c_ki, c_kd, n_angle, 0);
        pid_o = constrain(pid_o, -100, 100);

        int ms1 = (rvx * cosM1 + rvy * sinM1 + rwz - pid_o);
        int ms2 = (rvx * cosM2 + rvy * sinM2 + rwz - pid_o);
        int ms3 = (rvx * cosM3 + rvy * sinM3 + rwz - pid_o);

        m1.setSpeed(constrain(ms1, -255, 255));
        m2.setSpeed(constrain(ms2, -255, 255));
        m3.setSpeed(constrain(ms3, -255, 255));
        ////////////////////////////////////////////////////////
        if (mode % 2 == 0) {
          if (ps5.Triangle()) espSerial.println("sta");
          else if (ps5.Cross()) espSerial.println("stc");
          else espSerial.println("S");
          if (ps5.Square()) st.setSpeed(255);
          else if (ps5.Circle()) st.setSpeed(-255);
          else st.setSpeed(0);

          ps5.Touchpad() ? espSerial.println("P1") : espSerial.println("p1");
        } else {
          if (ps5.Cross()) espSerial.println("kff");
          else if (ps5.Triangle()) espSerial.println("kfb");
          else espSerial.println("S");

          ps5.Touchpad() ? espSerial.println("P2") : espSerial.println("p2");
        }

        ////////////////////////////////////////////////////////
        ps = ps5.RStickY();
        if (ps > 30) {
          dir.writeMicroseconds(2000);
        } else {
          dir.writeMicroseconds(1000);
        }
        if (abs(ps) > 30) {
          x = map(abs(ps), 30, 127, 1050, 1900);
          esc.writeMicroseconds(x);
        } else {
          esc.writeMicroseconds(1000);
        }
      } else {
        digitalWrite(ch_relay, LOW);
        esc.writeMicroseconds(1000);
        dir.writeMicroseconds(1000);
        stopAll();
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
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
      if (angle > 180) angle -= 360;

      input_angle = angle;

      if (is_rotating) {

        c_set = angle;
        n_angle = 0;
        pid_integral = 0;
        pid_prev_err = 0;
        stable_timer = millis();
      }

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
          } else {

            n_angle = 0;
            c_set = angle;
          }
        }

        last_angle = angle;
      }
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// int pid(float kp, float ki, float kd, int av, int sp) {
//   int err = av - sp;
//   float prop = err * kp;
//   pid_integral += err * ki;
//   pid_integral = constrain(pid_integral, -100.0, 100.0);
//   float deriv = (err - pid_prev_err) * kd;
//   pid_prev_err = err;
//   return (int)(prop + pid_integral + deriv);
// }
////========================= Speed adaptive ==================================
int pid(float kp, float ki, float kd, int av, int sp) {
    float speed_mag = (float)(abs(curr_vx) + abs(curr_vy)) / (2.0f * 255.0f);
  speed_mag = constrain(speed_mag, 0.0f, 1.0f);

  float adaptive_kp = kp * (1.0f - 0.5f * speed_mag);  
  float adaptive_ki = ki * (1.0f - 0.8f * speed_mag);  
  float adaptive_kd = kd * (1.0f - 0.3f * speed_mag); 

  int err = av - sp;
  float prop  = err * adaptive_kp;
  pid_integral += err * adaptive_ki;
  pid_integral  = constrain(pid_integral, -100.0f, 100.0f);
  float deriv = (err - pid_prev_err) * adaptive_kd;
  pid_prev_err  = err;
  return (int)(prop + pid_integral + deriv);
}

float acceleration(float current, float target, float factor) {
  return current + (target - current) * factor;
}

void stopAll() {
  m1.setSpeed(0);
  m2.setSpeed(0);
  m3.setSpeed(0);
  esc.writeMicroseconds(1000);
  dir.writeMicroseconds(1000);
  st.setSpeed(0);
}

void Motordebug() {
  m1.setSpeed(0);
  m2.setSpeed(0);
  m3.setSpeed(0);
  delay(500);

  m1.setSpeed(100);
  m2.setSpeed(0);
  m3.setSpeed(0);
  delay(500);

  m1.setSpeed(-100);
  m2.setSpeed(0);
  m3.setSpeed(0);
  delay(500);

  m1.setSpeed(0);
  m2.setSpeed(100);
  m3.setSpeed(0);
  delay(500);

  m1.setSpeed(0);
  m2.setSpeed(-100);
  m3.setSpeed(0);
  delay(500);

  m1.setSpeed(0);
  m2.setSpeed(0);
  m3.setSpeed(100);
  delay(500);

  m1.setSpeed(0);
  m2.setSpeed(0);
  m3.setSpeed(-100);
  delay(500);
}

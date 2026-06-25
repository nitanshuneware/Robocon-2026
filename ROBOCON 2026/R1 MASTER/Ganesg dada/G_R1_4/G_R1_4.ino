#include <ps5Controller.h>
#include <CytronMotorDriver.h>
#include "HardwareSerial.h"
#include <ESP32Servo.h>
#include "BNO055_support.h"
#include <Wire.h>
#include <math.h>
#include <TFminiS.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

#if defined(__AVR_ATmega2560__)
#define tfSerial Serial1
TFminiS tfmini(tfSerial);
#elif defined(ESP32)
HardwareSerial tfSerial(2);
TFminiS tfmini(tfSerial);
#endif

const int ch_relay = 19;
int throttlePin = 5;
int dirPin = 18;

Servo esc;
Servo dir;

int stp = 1500;
int x = 1500;
int ps;
float dist;
int h_pid_o;
CytronMD m1(PWM_DIR, 32, 33);  // Front wheel
CytronMD m2(PWM_DIR, 27, 14);  // Right wheel
CytronMD m3(PWM_DIR, 12, 13);  // Left wheel

HardwareSerial espSerial(0);

struct bno055_t myBNO;
struct bno055_euler myEulerData;
unsigned long lastTime = 0;

int spd = 90;
int spd2 = 200;
int vx = 0, vy = 0, wz = 0;
float curr_vx = 0, curr_vy = 0, curr_wz = 0;

const float accel = 0.08;

// Heading PID
float c_kp = 3.0, c_ki = 0.003, c_kd = 3.0;
float s = 60.7;
volatile int c_set = 0;
volatile int input_angle = 0;
volatile int n_angle = 0;
float pid_integral = 0;
int pid_prev_err = 0;
int pid_o = 0;

// Height PID
float h_kp = 150, h_ki = 0.5, h_kd = 10;
float h_integral = 0;
int h_prev_err = 0;

int h_set1 = 31;
int h_set2 = 52;

// Height toggle state
int h1 = 0, h2 = 0,h3 = 0;
int curr_h1 = 0, pre_h1 = 0;
int curr_h2 = 0, pre_h2 = 0;
int curr_h3 = 0, pre_h3 = 0;

int start = 0;
int mode = 0;
int ch_mode = 0;

int curr_srt, pre_srt;
int curr_r, pre_r;
int curr_l, pre_l;
int curr_mode, pre_mode;
int curr_ch_mode, pre_ch_mode;

bool was_rotating = false;
int prev_wz = 0;
unsigned long brake_timer = 0;
bool braking = false;
volatile bool is_rotating = false;

const float ANGLE_M1 = 90.0;   // FRONT
const float ANGLE_M2 = -30.0;  // RIGHT
const float ANGLE_M3 = 210.0;  // LEFT

float cosM1, sinM1;
float cosM2, sinM2;
float cosM3, sinM3;

void Maincode(void* parameter);
void Orientation(void* parameter);
void chmode1();
void chmode2();
int h_pid(float kp, float ki, float kd, int av, int sp);
int pid(float kp, float ki, float kd, int av, int sp);
float acceleration(float current, float target, float factor);
void stopAll();
void Motordebug();

void setup() {
  // Serial.begin(115200);
  espSerial.begin(250000);
  tfSerial.begin(115200);

  pinMode(ch_relay, OUTPUT);
  pinMode(2, OUTPUT);
  ps5.begin("A0:FA:9C:0D:D5:BE");

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
      digitalWrite(2, HIGH);

      if (ps5.PSButton() && ps5.Options()) {
        stopAll();
        vTaskDelay(200 / portTICK_PERIOD_MS);
        continue;
      }

      curr_srt = ps5.Share();
      if (curr_srt && !pre_srt) start++;
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

        spd = constrain(spd, -255, 255);

        ps = ps5.RStickY();

        (ch_mode % 2 == 0) ? chmode1() : chmode2();

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

        if (mode % 2 == 0) {
          // --- RC throttle / steering mode ---
          if (ps5.LStickY() > 30) espSerial.println("sta");
          else if (ps5.LStickY() < -30) espSerial.println("stc");
          else if (ps5.Square()) espSerial.println("stp");
          else if (ps5.Circle()) espSerial.println("sts");
          else espSerial.println("S");

          ps5.Touchpad() ? espSerial.println("P1") : espSerial.println("p1");

          curr_h3 = ps5.Triangle();
          if (curr_h3 && !pre_h3) h3++;
          pre_h3 = curr_h3;
          if (h3 % 2 == 1) {
            h_pid_o = h_pid(h_kp, h_ki, h_kd, dist, s);
            if (dist < s)
              dir.writeMicroseconds(2000);
            else
              dir.writeMicroseconds(1000);
            esc.writeMicroseconds(h_pid_o);

          } else {
            ps = ps5.RStickY();
            if (ps > 30) {
              dir.writeMicroseconds(2000);
            } else {
              dir.writeMicroseconds(1000);
            }
            if (abs(ps) > 30) {
              x = map(abs(ps), 30, 127, 1050, 2000);
              esc.writeMicroseconds(x);
            } else {
              esc.writeMicroseconds(1000);
            }
          }

        } else {

          curr_h1 = ps5.Square();
          // curr_h2 = ps5.Circle();

          if (curr_h1 && !pre_h1) h1 = (h1 + 1) % 3;
          // if (curr_h2 && !pre_h2) h2++;

          pre_h1 = curr_h1;
          // pre_h2 = curr_h2;

          if (ps5.Cross()) espSerial.println("kff");
          else if (ps5.Triangle()) espSerial.println("kfb");
          else espSerial.println("S");

          ps5.Touchpad() ? espSerial.println("P2") : espSerial.println("p2");

          int h_set = -1;

          if (h1 == 1) h_set = h_set1;
          else if (h1 == 2) h_set = h_set2;
          else if (h1 == 0) {
            ps = ps5.RStickY();

            if (ps > 30)
              dir.writeMicroseconds(2000);
            else
              dir.writeMicroseconds(1000);

            if (abs(ps) > 30) {
              x = map(abs(ps), 30, 127, 1050, 2000);
              esc.writeMicroseconds(x);
            } else {
              esc.writeMicroseconds(1000);
            }
          }


          if (h_set != -1) {

            h_pid_o = h_pid(h_kp, h_ki, h_kd, dist, h_set);

            if (dist < h_set)
              dir.writeMicroseconds(2000);
            else
              dir.writeMicroseconds(1000);
            // if(h_pid_o < 0)(h_pid_o = 1500 + h_pid_o);
            esc.writeMicroseconds(h_pid_o);
          }

          else {

            ps = ps5.RStickY();

            if (ps > 30)
              dir.writeMicroseconds(2000);
            else
              dir.writeMicroseconds(1000);

            if (abs(ps) > 30) {
              x = map(abs(ps), 30, 127, 1050, 2000);
              esc.writeMicroseconds(x);
            } else {
              esc.writeMicroseconds(1000);
            }
          }
        }
      }

    } else {
      digitalWrite(ch_relay, LOW);
      digitalWrite(2, LOW);
      esc.writeMicroseconds(1000);
      dir.writeMicroseconds(1000);
      stopAll();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void chmode1() {
  if (ps5.R2Value() > 30 || ps5.L2Value() > 30 || ps5.Right() || ps5.Left() || ps5.Up() || ps5.Down()) {
    if (ps5.R2Value() > 30 || ps5.L2Value() > 30) {
      wz = map(ps5.R2Value() - ps5.L2Value(), -255, 255, -60,60);
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
    if (ps5.Up()) vx = -spd;
    else if (ps5.Down()) vx = spd;
    else vx = 0;
    if (ps5.Right()) vy = -spd;
    else if (ps5.Left()) vy = spd;
    else vy = 0;
  } else {
    vx = 0;
    vy = 0;
    wz = 0;
  }
}

void chmode2() {
  if (ps5.R2Value() > 30 || ps5.L2Value() > 30 || ps5.Right() || ps5.Left() || ps5.Up() || ps5.Down()) {
    if (ps5.R2Value() > 30 || ps5.L2Value() > 30) {
      wz = map(ps5.R2Value() - ps5.L2Value(), -255, 255, -60, 60);
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
    if (ps5.Up()) vx = -spd2;
    else if (ps5.Down()) vx = spd2;
    else vx = 0;
    if (ps5.Right()) vy = -spd2;
    else if (ps5.Left()) vy = spd2;
    else vy = 0;
  } else {
    vx = 0;
    vy = 0;
    wz = 0;
  }
}

void Orientation(void* pvParameters) {
  static int last_angle = 0;
  static unsigned long stable_timer = 0;

  while (1) {
    tfmini.readSensor();
    dist = tfmini.getDistance();
dist = dist/10;
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
        stable_timer = millis();
      } else {
        int delta = abs(angle - last_angle);
        if (delta > 180) delta = 360 - delta;

        if (delta > 1) {
          c_set = angle;
          n_angle = 0;
          stable_timer = millis();
        } else {
          if (millis() - stable_timer > 200) {
            int diff = angle - c_set;
            if (diff < -180) n_angle = diff + 360;
            else if (diff > 180) n_angle = diff - 360;
            else n_angle = diff;
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

int pid(float kp, float ki, float kd, int av, int sp) {
  float speed_mag = (float)(abs(curr_vx) + abs(curr_vy)) / (2.0f * 255.0f);
  speed_mag = constrain(speed_mag, 0.0f, 1.0f);

  float adaptive_kp = kp * (1.0f - 0.5f * speed_mag);
  float adaptive_ki = ki * (1.0f - 0.8f * speed_mag);
  float adaptive_kd = kd * (1.0f - 0.3f * speed_mag);

  int err = av - sp;
  float prop = err * adaptive_kp;
  pid_integral += err * adaptive_ki;
  pid_integral = constrain(pid_integral, -100.0f, 100.0f);
  float deriv = (err - pid_prev_err) * adaptive_kd;
  pid_prev_err = err;
  return (int)(prop + pid_integral + deriv);
}

int h_pid(float kp, float ki, float kd, int av, int sp) {
  int err = sp - av;
  float prop = err * kp;
  h_integral += err * ki;
  h_integral = constrain(h_integral, -500.0f, 500.0f);
  float deriv = (err - h_prev_err) * kd;
  h_prev_err = err;
  int pidOut = constrain((int)(prop + h_integral + deriv), -1000, 1000);
  (pidOut < 0) ? pidOut = -pidOut : pidOut = pidOut;
  return map(pidOut, 0, 1000, 1000, 2000);
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

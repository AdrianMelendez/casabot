/*
 * casabot base controller - ESP32 (Arduino core 3.x)
 *
 * Owns the two motors, their quadrature encoders and the MPU6050.
 * Talks to base_driver.py over USB serial at 115200 baud:
 *
 *   in   v <left_rad_s> <right_rad_s>
 *   out  o <left_ticks> <right_ticks>                 @ 50 Hz
 *   out  i <ax> <ay> <az> <gx> <gy> <gz>              @ 50 Hz, SI units
 *
 * Motor driver: TB6612FNG. Swap the pins below to match your wiring.
 */

#include <Wire.h>

// ---- pins ------------------------------------------------------------------
const int PIN_L_PWM = 25, PIN_L_IN1 = 26, PIN_L_IN2 = 27;
const int PIN_R_PWM = 33, PIN_R_IN1 = 32, PIN_R_IN2 = 14;
const int PIN_STBY  = 12;
const int PIN_L_ENC_A = 34, PIN_L_ENC_B = 35;
const int PIN_R_ENC_A = 36, PIN_R_ENC_B = 39;
const int PIN_SDA = 21, PIN_SCL = 22;

// ---- tuning ----------------------------------------------------------------
// TICKS_PER_REV counts every edge this sketch reacts to, at the OUTPUT shaft.
// For an 11-PPR motor encoder behind a 30:1 gearbox, counting one edge:
// 11 * 30 * 4 = 1320 if you count all four quadrature edges, 1320/4 if you
// count one. This sketch counts one edge per channel A transition.
const float TICKS_PER_REV = 1320.0f;
const float MAX_WHEEL_RAD_S = 12.0f;   // clamp, protects the gearbox
const float KP = 2.0f, KI = 8.0f, KD = 0.0f;
const uint32_t CONTROL_PERIOD_US = 5000;   // 200 Hz PID
const uint32_t REPORT_PERIOD_US  = 20000;  // 50 Hz telemetry
const uint32_t CMD_TIMEOUT_US    = 500000; // stop if ROS goes quiet

const int MPU_ADDR = 0x68;
const float ACCEL_SCALE = 9.80665f / 16384.0f;         // +/- 2 g -> m/s^2
const float GYRO_SCALE  = (PI / 180.0f) / 131.0f;      // +/- 250 dps -> rad/s

// ---- state -----------------------------------------------------------------
volatile long ticksLeft = 0, ticksRight = 0;
long lastTicksLeft = 0, lastTicksRight = 0;

float targetLeft = 0.0f, targetRight = 0.0f;   // rad/s
float integralLeft = 0.0f, integralRight = 0.0f;
float prevErrLeft = 0.0f, prevErrRight = 0.0f;

uint32_t lastControlUs = 0, lastReportUs = 0, lastCmdUs = 0;
char rxBuf[64];
uint8_t rxLen = 0;

void IRAM_ATTR onLeftA()  { ticksLeft  += digitalRead(PIN_L_ENC_B) ? 1 : -1; }
void IRAM_ATTR onRightA() { ticksRight += digitalRead(PIN_R_ENC_B) ? -1 : 1; }

void setMotor(int pwmPin, int in1, int in2, float effort) {
  effort = constrain(effort, -1.0f, 1.0f);
  digitalWrite(in1, effort >= 0);
  digitalWrite(in2, effort < 0);
  ledcWrite(pwmPin, (uint32_t)(fabsf(effort) * 255.0f));
}

float pid(float target, float measured, float dt,
          float &integral, float &prevErr) {
  float err = target - measured;
  integral += err * dt;
  integral = constrain(integral, -1.0f / KI, 1.0f / KI);   // anti-windup
  float derivative = (err - prevErr) / dt;
  prevErr = err;
  // Feedforward on target does most of the work; PID only trims it.
  return target / MAX_WHEEL_RAD_S + KP * err / MAX_WHEEL_RAD_S
         + KI * integral + KD * derivative;
}

void mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // wake up
  Wire.endTransmission();
}

bool mpuRead(float *out) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) return false;

  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (Wire.read() << 8) | Wire.read();

  out[0] = raw[0] * ACCEL_SCALE;   // ax
  out[1] = raw[1] * ACCEL_SCALE;   // ay
  out[2] = raw[2] * ACCEL_SCALE;   // az   (raw[3] is temperature)
  out[3] = raw[4] * GYRO_SCALE;    // gx
  out[4] = raw[5] * GYRO_SCALE;    // gy
  out[5] = raw[6] * GYRO_SCALE;    // gz
  return true;
}

void handleLine(char *line) {
  if (line[0] != 'v') return;
  float l, r;
  if (sscanf(line + 1, "%f %f", &l, &r) != 2) return;
  targetLeft  = constrain(l, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
  targetRight = constrain(r, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
  lastCmdUs = micros();
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen) { rxBuf[rxLen] = '\0'; handleLine(rxBuf); rxLen = 0; }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;   // overlong garbage, drop it rather than wrap
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_L_IN1, OUTPUT); pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_IN1, OUTPUT); pinMode(PIN_R_IN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);  digitalWrite(PIN_STBY, HIGH);
  ledcAttach(PIN_L_PWM, 20000, 8);   // 20 kHz, above hearing
  ledcAttach(PIN_R_PWM, 20000, 8);

  pinMode(PIN_L_ENC_A, INPUT); pinMode(PIN_L_ENC_B, INPUT);
  pinMode(PIN_R_ENC_A, INPUT); pinMode(PIN_R_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_L_ENC_A), onLeftA,  RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_R_ENC_A), onRightA, RISING);

  mpuInit();
  lastControlUs = lastReportUs = lastCmdUs = micros();
}

void loop() {
  readSerial();
  uint32_t now = micros();

  if (now - lastCmdUs > CMD_TIMEOUT_US) {
    targetLeft = targetRight = 0.0f;
  }

  if (now - lastControlUs >= CONTROL_PERIOD_US) {
    float dt = (now - lastControlUs) * 1e-6f;
    lastControlUs = now;

    noInterrupts();
    long tl = ticksLeft, tr = ticksRight;
    interrupts();

    float measLeft  = (tl - lastTicksLeft)  * TWO_PI / TICKS_PER_REV / dt;
    float measRight = (tr - lastTicksRight) * TWO_PI / TICKS_PER_REV / dt;
    lastTicksLeft = tl;
    lastTicksRight = tr;

    setMotor(PIN_L_PWM, PIN_L_IN1, PIN_L_IN2,
             pid(targetLeft, measLeft, dt, integralLeft, prevErrLeft));
    setMotor(PIN_R_PWM, PIN_R_IN1, PIN_R_IN2,
             pid(targetRight, measRight, dt, integralRight, prevErrRight));
  }

  if (now - lastReportUs >= REPORT_PERIOD_US) {
    lastReportUs = now;

    noInterrupts();
    long tl = ticksLeft, tr = ticksRight;
    interrupts();
    Serial.printf("o %ld %ld\n", tl, tr);

    float imu[6];
    if (mpuRead(imu)) {
      Serial.printf("i %.4f %.4f %.4f %.5f %.5f %.5f\n",
                    imu[0], imu[1], imu[2], imu[3], imu[4], imu[5]);
    }
  }
}

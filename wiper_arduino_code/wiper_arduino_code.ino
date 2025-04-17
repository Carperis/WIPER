// Include necessary libraries
#include <Servo.h>
#include "Wire.h"

// Motor and encoder pins
#define ENCA_M1 2
#define ENCB_M1 A2
#define ENCA_M2 3
#define ENCB_M2 A3
#define PWM_M1 11
#define AIN2_M1 12
#define AIN1_M1 10
#define STBY 9
#define PWM_M2 6
#define BIN2_M2 7
#define BIN1_M2 8

#define IN1 AIN1_M1
#define IN2 AIN2_M1
#define ENA PWM_M1

#define IN3 BIN1_M2
#define IN4 BIN2_M2
#define ENB PWM_M2

// Servo pins
#define servo1pin A0
#define servo2pin A1

// Battery
#define btyPin A6

// Motor and encoder variables
volatile int posi_M1 = 0;
volatile int posi_M2 = 0;
double SetpointM1, InputM1, OutputM1, TargetM1 = 0;
double SetpointM2, InputM2, OutputM2, TargetM2 = 0;
float prevT = 0;
float posiprev_M1 = 0;
float posiprev_M2 = 0;
float RPM_M1;
float RPM_M2;

const int offsetA = 1;
const int offsetB = 1;
float RPM_default = 50;
float RPM_turn = 30;
int driveM1, driveM2;
float instantDistance = 0;
float startTime, endTime;

float lastRPM1 = 0;
float lastRPM2 = 0;
float imuX = 0, imuY = 0;
float vx = 0, vy = 0;
unsigned long lastIMUTime = 0;
float dt = 0;

// Servo variables
Servo servo1;
Servo servo2;
int mode3_offset = 65;
int mode2_offset = 75;

// Accelerometer variables
const int MPU_ADDR = 0x68;
double ax, ay, az, pitch, roll, degToX;
int16_t accelerometer_x, accelerometer_y, accelerometer_z;

// Battery variables
float batteryVoltage = 0;
float maxVoltage = 4;

// Command variables
int mode = 1;
bool power = false;
double carx = 0;
double cary = 0;
double deg = 0;
double targetx = 0;
double targety = 0;
float targetAngle, targetDistance, currDistance = 0;

double r1 = 0.0;
double r2 = 0.0;
int duration = 0;

bool isRunning = false;
unsigned long commandStartTime = 0;

void setup() {
  Serial.begin(9600);
  servo1.attach(servo1pin);
  servo2.attach(servo2pin);

  pinMode(ENCA_M1, INPUT_PULLUP);
  pinMode(ENCA_M2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_M1), readEncoderM1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2), readEncoderM2, RISING);
  
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(AIN1_M1, OUTPUT);
  pinMode(AIN2_M1, OUTPUT);
  pinMode(PWM_M1, OUTPUT);
  pinMode(BIN1_M2, OUTPUT);
  pinMode(BIN2_M2, OUTPUT);
  pinMode(PWM_M2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // This MUST be HIGH to enable motors
  
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  startTime = millis();
  //calculateSpeed();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
  
    if (cmd.length() > 0) {
      //Serial.println(cmd);
  
      // Parse first number (r1)
      r1 = cmd.substring(0, cmd.indexOf(',')).toDouble();
      cmd.remove(0, cmd.indexOf(',') + 1);
  
      // Parse second number (r2)
      if (cmd.indexOf(',') != -1) {
        r2 = cmd.substring(0, cmd.indexOf(',')).toDouble();
        cmd.remove(0, cmd.indexOf(',') + 1);
  
        // Parse third value (duration)
        // Parse third value (duration)
        duration = cmd.toInt();
        commandStartTime = millis();
        isRunning = true;

        // Set motors immediately
        lastRPM1 = r1;
        lastRPM2 = r2;
  
        // Run motors for duration
        // setMotors((int)r1, (int)r2);
        // delay(duration);
        // setMotors(0, 0);

      } else {
        // Only r1 and r2 provided, no duration
        r2 = cmd.toDouble();
        lastRPM1 = r1;
        lastRPM2 = r2;
        isRunning = false;
      }
      //Serial.print("Parsed r1: "); Serial.println(r1);
      //Serial.print("Parsed r2: "); Serial.println(r2);
      //Serial.print("Duration: "); Serial.println(duration);
    }
  }
    if (isRunning && duration > 0) {
      if (millis() - commandStartTime >= duration) {
        lastRPM1 = 0;
        lastRPM2 = 0;
        isRunning = false;
      }
    }
  
  setMotors(lastRPM1, lastRPM2);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();
  ax = accelerometer_x / 16384.0 * 9.81;
  ay = accelerometer_y / 16384.0 * 9.81;
  az = accelerometer_z / 16384.0 * 9.81;
  pitch = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / PI);
  roll = atan2(ax, sqrt(ay * ay + az * az)) * (180.0 / PI);

  unsigned long now = millis();
  dt = (now - lastIMUTime) / 1000.0;
  lastIMUTime = now;

  vx += ax * dt;
  vy += ay * dt;
  imuX += vx * dt;
  imuY += vy * dt;

  degToX = (ax > 0) ? -(pitch - 90) : (pitch - 90);

  if (mode == 1) {
    servo1.write(90);
    servo2.write(90);
  } else if (mode == 2) {
    servo1.write(90 + mode2_offset);
    servo2.write(90 - mode2_offset);
  } else if (mode == 3) {
    servo1.write(90 - mode3_offset);
    servo2.write(90 + mode3_offset);
  }

  int analogValue = analogRead(btyPin);
  float voltage = analogValue * (5.0 / 1023.0);
  float voltageOffset = 2.6;
  batteryVoltage = voltage * 3.0 - voltageOffset;
  static int counter = 0;
  
  // Guard bad IMU reads
  if (isnan(imuX)) imuX = 0;
  if (isnan(imuY)) imuY = 0;
  if (isnan(dt)) dt = 0;
  
  reportData();
}

void reportData() {
    Serial.print(ax); Serial.print(" ");
    Serial.print(ay); Serial.print(" ");
    Serial.print(az); Serial.print(" ");
    Serial.println(dt, 5);  // precise dt with 5 decimal places
}

void readEncoderM1() {
  int a = digitalRead(ENCA_M1);
  float analog_b = analogRead(ENCB_M1);
  int b = (analog_b > 512) ? 1 : 0;
  if (a == HIGH && b == HIGH) posi_M1++;
  else if (a == HIGH && b == LOW) posi_M1--;
}

void readEncoderM2() {
  int a = digitalRead(ENCA_M2);
  float analog_b = analogRead(ENCB_M2);
  int b = (analog_b > 512) ? 1 : 0;
  if (a == HIGH && b == HIGH) posi_M2++;
  else if (a == HIGH && b == LOW) posi_M2--;
}

void calculateSpeed() {
  float currT = micros();
  float deltaTime = (currT - prevT) / 1000000.0;
  RPM_M1 = ((posi_M1 - posiprev_M1) / 700.0) * (1 / deltaTime) * 60;
  RPM_M2 = -((posi_M2 - posiprev_M2) / 700.0) * (1 / deltaTime) * 60;
  posiprev_M1 = posi_M1;
  posiprev_M2 = posi_M2;
  prevT = currT;

  float maximum_limit = 251;
  InputM1 = map(RPM_M1, -maximum_limit, maximum_limit, 0, 510);
  InputM2 = map(RPM_M2, -maximum_limit, maximum_limit, 0, 510);
}

void setMotors(int rpm1, int rpm2) {
  // === Motor 1 ===
  int pwm1 = constrain(map(abs(rpm1), 0, 300, 0, 255), 0, 255);
  if (rpm1 >= 0) {
    digitalWrite(AIN1_M1, HIGH);
    digitalWrite(AIN2_M1, LOW);
  } else {
    digitalWrite(AIN1_M1, LOW);
    digitalWrite(AIN2_M1, HIGH);
  }
  analogWrite(PWM_M1, pwm1);

  // === Motor 2 ===
  int pwm2 = constrain(map(abs(rpm2), 0, 300, 0, 255), 0, 255);
  if (rpm2 >= 0) {
    digitalWrite(BIN1_M2, HIGH);
    digitalWrite(BIN2_M2, LOW);
  } else {
    digitalWrite(BIN1_M2, LOW);
    digitalWrite(BIN2_M2, HIGH);
  }
  analogWrite(PWM_M2, pwm2);
}

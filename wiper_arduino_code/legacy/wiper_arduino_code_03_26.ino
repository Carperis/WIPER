// Include necessary libraries
#include <PID_v1.h>
#include <SparkFun_TB6612.h>
#include <Servo.h>
#include <SoftwareSerial.h>
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

// Servo pins
#define servo1pin A0
#define servo2pin A1

// Battery
#define btyPin A6

// Motor and encoder variables
volatile int posi_M1 = 0;
volatile int posi_M2 = 0;
// volatile float time = 0;
double SetpointM1, InputM1, OutputM1, TargetM1 = 0;
double SetpointM2, InputM2, OutputM2, TargetM2 = 0;
float prevT = 0;       // Previous time
float posiprev_M1 = 0; // Previous position for motor 1
float posiprev_M2 = 0; // Previous position for motor 2
float RPM_M1;
float RPM_M2;
double Kp = 0, Ki = 10, Kd = 0;
const int offsetA = 1;
const int offsetB = 1;
float RPM_default = 50;
float RPM_turn = 30;
int driveM1, driveM2;
float instantDistance = 0;
float startTime, endTime;
PID myPIDM1(&InputM1, &OutputM1, &SetpointM1, Kp, Ki, Kd, DIRECT);
PID myPIDM2(&InputM2, &OutputM2, &SetpointM2, Kp, Ki, Kd, DIRECT);
Motor motor1 = Motor(AIN1_M1, AIN2_M1, PWM_M1, offsetA, STBY);
Motor motor2 = Motor(BIN1_M2, BIN2_M2, PWM_M2, offsetB, STBY);

// Servo variables
Servo servo1;
Servo servo2;
int mode3_offset = 65;
int mode2_offset = 75;

// Accelerometer variables
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050.
double ax, ay, az, pitch, roll, degToX;
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // Variables for accelerometer raw data

// Bluetooth variables
int TX_pin = 5;
int RX_pin = 4;
SoftwareSerial BTserial(TX_pin, RX_pin);

// Battery variables
float batteryVoltage = 0;
float maxVoltage = 4; // V

// Command variables

int mode = 1; //1: down, 2: up, 3: disengage
bool power = false;
double carx = 0;
double cary = 0;
double deg = 0;
double targetx = 0;
double targety = 0;
float targetAngle, targetDistance, currDistance = 0;

// Function prototypes
void readEncoderM1();
void readEncoderM2();
void calculateSpeed();
void reportData(bool isBluetooth);
void keyboardControl(bool isBluetooth);
void commandControl(bool isBluetooth);
void adjustMotors();

void setup()
{
    Serial.begin(9600);
    BTserial.begin(9600);

    servo1.attach(servo1pin);
    servo2.attach(servo2pin);

    // Encoder inputs
    pinMode(ENCA_M1, INPUT_PULLUP);
    pinMode(ENCA_M2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCA_M1), readEncoderM1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA_M2), readEncoderM2, RISING);
    SetpointM1 = TargetM1 + 255;
    SetpointM2 = TargetM2 + 255;
    myPIDM1.SetMode(AUTOMATIC);
    myPIDM2.SetMode(AUTOMATIC);

    Wire.begin();
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(0x6B);                 // PWR_MGMT_1 register
    Wire.write(0);                    // Set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
}

void loop()
{
    startTime = millis();

    SetpointM1 = TargetM1 + 255;
    SetpointM2 = TargetM2 + 255;

    calculateSpeed();
    if (power)
    {
        myPIDM1.Compute();
        myPIDM2.Compute();
        driveM1 = map(OutputM1, 0, 255, -255, 255);
        driveM2 = map(OutputM2, 0, 255, -255, 255);
        int minimum_limit = 10;
        if (abs(driveM1) < minimum_limit)
        {
            motor1.drive(0);
        }
        else
        {
            motor1.drive(driveM1);
        }
        if (abs(driveM2) < minimum_limit)
        {
            motor2.drive(0);
        }
        else
        {
            motor2.drive(driveM2);
        }
    }
    else
    {
        motor1.brake();
        motor2.brake();
        TargetM1 = 0;
        TargetM2 = 0;
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true); // Request a total of 14 registers
    accelerometer_x = Wire.read() << 8 | Wire.read();
    accelerometer_y = Wire.read() << 8 | Wire.read();
    accelerometer_z = Wire.read() << 8 | Wire.read();
    ax = accelerometer_x / 16384.0 * 9.81;
    ay = accelerometer_y / 16384.0 * 9.81;
    az = accelerometer_z / 16384.0 * 9.81;
    pitch = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / PI);
    roll = atan2(ax, sqrt(ay * ay + az * az)) * (180.0 / PI);
    if (ax > 0)
    {
        degToX = -(pitch - 90);
    }
    else
    {
        degToX = (pitch - 90);
    }

    if (mode == 1)
    {
        servo1.write(90);
        servo2.write(90);
    }
    else if (mode == 2)
    {
        servo1.write(90 + mode2_offset);
        servo2.write(90 - mode2_offset);
    }
    else if (mode == 3)
    {
        servo1.write(90 - mode3_offset);
        servo2.write(90 + mode3_offset);
    }

    int analogValue = analogRead(btyPin); // Read analog value from A6 pin
    float voltage = analogValue * (5.0 / 1023.0);   // Convert analog value to voltage
    float voltageOffset = 2.6; // when complete wireless
    batteryVoltage = voltage * 3.0 - voltageOffset;

    reportData(false);  // Send data via Bluetooth
    // keyboardControl(false);
    commandControl(false);
}

void keyboardControl(bool isBluetooth)
{
    char inChar = '\0';

    if (isBluetooth)
    {
        if (BTserial.available() > 0)
        {
            inChar = BTserial.read();
        }
    }
    else
    {
        if (Serial.available() > 0)
        {
            inChar = Serial.read();
        }
    }

    // Serial.println(inChar);

    switch (inChar)
    {
    case 'w':
        TargetM1 = RPM_default;
        TargetM2 = RPM_default;
        break;
    case 's':
        TargetM1 = -RPM_default;
        TargetM2 = -RPM_default;
        break;
    case 'a':
        TargetM1 = -RPM_default;
        TargetM2 = RPM_default;
        break;
    case 'd':
        TargetM1 = RPM_default;
        TargetM2 = -RPM_default;
        break;
    case 'x':
        TargetM1 = 0;
        TargetM2 = 0;
        break;
    case '1':
        mode = 1;
        break;
    case '2':
        mode = 2;
        break;
    case '3':
        mode = 3;
        break;
    default:;
    }
}

void commandControl(bool isBluetooth)
{
    String cmd = "";
    String values[5];

    if (isBluetooth)
    {
        if (BTserial.available())
        {
            cmd = BTserial.readStringUntil('\n');
        }
    }
    else
    {
        if (Serial.available())
        {
            cmd = Serial.readStringUntil('\n');
        }
    }

    if (cmd.length() > 0)
    {
        // Serial.println(cmd);

        // Extract carx, cary, and targetangle from the input string
        carx = cmd.substring(0, cmd.indexOf(',')).toDouble();
        cmd.remove(0, cmd.indexOf(',') + 1);
        cary = cmd.substring(0, cmd.indexOf('|')).toDouble();
        cmd.remove(0, cmd.indexOf('|') + 1);
        // deg = cmd.substring(0, cmd.indexOf('|')).toDouble();
        // cmd.remove(0, cmd.indexOf('|') + 1);
        targetx = cmd.substring(0, cmd.indexOf(',')).toDouble();
        cmd.remove(0, cmd.indexOf(',') + 1);
        targety = cmd.substring(0, cmd.indexOf('|')).toDouble();
        cmd.remove(0, cmd.indexOf('|') + 1);
        int mode_int = cmd.substring(0, cmd.indexOf('|')).toInt();
        cmd.remove(0, cmd.indexOf('|') + 1);
        int power_int = cmd.toInt();

        switch (mode_int)
        {
        case 1:
            mode = 1;
            break;
        case 2:
            mode = 2;
            break;
        case 3:
            mode = 3;
            break;
        default:;
        }

        if (power_int == 1)
        {
            power = true;
        }
        else
        {
            power = false;
        }

        // Calculate targetangle
        targetAngle = atan2(targety - cary, targetx - carx) * (180 / PI);
        targetDistance = sqrt(pow(targetx - carx, 2) + pow(targety - cary, 2));

        cmd = "";
        currDistance = 0;
    }

    adjustMotors();
}

void adjustMotors()
{
    float deg_tolerance = 10;
    float dis_tolerance = 0;
    float degDiff = targetAngle - degToX;
    float disDiff = targetDistance - currDistance;

    if (degDiff > 180)
    {
        degDiff -= 360;
    }
    else if (degDiff < -180)
    {
        degDiff += 360;
    }

    if (abs(degDiff) > deg_tolerance && disDiff > dis_tolerance)
    {
        if (degDiff > 0) // counterclockwise
        {
            TargetM1 = -RPM_turn;
            TargetM2 = RPM_turn;
        }
        else // clockwise
        {
            TargetM1 = RPM_turn;
            TargetM2 = -RPM_turn;
        }
    }
    else if (disDiff > dis_tolerance)
    {
        endTime = millis();
        float wheel_radius = 0.0325;          // meters
        float speed_offset = 0.3; // -1~+1, neg->loss, pos->gain
        float avg_speed = (RPM_M1 + RPM_M2) / 2 * 2 * PI * wheel_radius / 60; // m/s
        avg_speed = avg_speed*(1+speed_offset);
        instantDistance = avg_speed * (endTime - startTime) / 1000;
        currDistance += instantDistance;

        TargetM1 = RPM_default;
        TargetM2 = RPM_default;
    }
    else
    {
        // motor1.brake();
        // motor2.brake();
        TargetM1 = 0;
        TargetM2 = 0;
    }
}

void reportData(bool isBluetooth)
{
    char buffer[200];    // Define a buffer to hold the formatted data
    char tempBuffer[20]; // Temporary buffer to hold the converted doubleing-point numbers

    // Convert num to string with 6 characters and 2 decimal places
    int numLen = 4;
    int numDec = 1;

    dtostrf(startTime, numLen + 2, 1, tempBuffer);
    snprintf(buffer, sizeof(buffer), "%ss %s ", tempBuffer, power ? "ON" : "OFF");
    
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "M%d ", mode);
    dtostrf(batteryVoltage, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%sV | ", tempBuffer);
    dtostrf(carx, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "(%sm,", tempBuffer);
    dtostrf(cary, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%sm)=>", tempBuffer);
    // dtostrf(deg, numLen, numDec, tempBuffer);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%s°, ", tempBuffer);
    dtostrf(targetx, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "(%sm, ", tempBuffer);
    dtostrf(targety, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%sm) | ", tempBuffer);

    dtostrf(RPM_M1, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "L %s =[%4d]=>", tempBuffer, driveM1);
    dtostrf(TargetM1, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%s ", tempBuffer);
    dtostrf(RPM_M2, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "R %s =[%4d]=>", tempBuffer, driveM2);
    dtostrf(TargetM2, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%s | ", tempBuffer);
    dtostrf(targetAngle, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "TarDeg %s ", tempBuffer);
    dtostrf(targetDistance, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "TarDis %sm | ", tempBuffer);

    // dtostrf(pitch, numLen, numDec, tempBuffer);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "Pitch: %s° ", tempBuffer);
    // dtostrf(roll, numLen, numDec, tempBuffer);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "Roll: %s° ", tempBuffer);
    dtostrf(degToX, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "CurrDeg: %s ", tempBuffer);
    dtostrf(currDistance, numLen, numDec, tempBuffer);
    snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "CurrDis: %sm\n", tempBuffer);
    
    // Print the formatted data from the buffer 
    if (isBluetooth)
    {
        BTserial.print(buffer);
        // BTserial.print("Mode: ");
        // BTserial.print(mode);
        // BTserial.print("\t");

        // BTserial.print("Left RPM: ");
        // BTserial.print(RPM_M1);
        // BTserial.print("->");
        // BTserial.print(TargetM1);
        // BTserial.print("\t");

        // BTserial.print("Right RPM: ");
        // BTserial.print(RPM_M2);
        // BTserial.print("->");
        // BTserial.print(TargetM2);
        // BTserial.print("\t");

        // BTserial.print("Pitch: ");
        // BTserial.print(pitch);
        // BTserial.print("\t");
        // BTserial.print("Roll: ");
        // BTserial.print(roll);
        // BTserial.println();
    }
    else
    {
        Serial.print(buffer);
    }
}

// Interrupt service routines for encoders
void readEncoderM1()
{
    int a = digitalRead(ENCA_M1); // Read the state of the encoder A pin
    float analog_b = analogRead(ENCB_M1);
    int b;
    if (analog_b > 512)
    {
        b = 1;
    }
    else
    {
        b = 0;
    }

    if (a == HIGH && b == HIGH)
    {              // If both encoder A and B pins are high
        posi_M1++; // Increment the position
    }
    else if (a == HIGH && b == LOW)
    {              // If encoder A pin is high and encoder B pin is low
        posi_M1--; // Decrement the position
    }
}

void readEncoderM2()
{
    int a = digitalRead(ENCA_M2); // Read the state of the encoder A pin
    float analog_b = analogRead(ENCB_M2);
    int b;
    if (analog_b > 512)
    {
        b = 1;
    }
    else
    {
        b = 0;
    }

    if (a == HIGH && b == HIGH)
    {              // If both encoder A and B pins are high
        posi_M2++; // Increment the position++
    }
    else if (a == HIGH && b == LOW)
    {              // If encoder A pin is high and encoder B pin is low
        posi_M2--; // Decrement the position --
    }
}

void calculateSpeed()
{
    float currT = micros();
    float deltaTime = (currT - prevT) / 1000000.0;                     // Convert to seconds
    RPM_M1 = ((posi_M1 - posiprev_M1) / 700.0) * (1 / deltaTime) * 60; // Calculate RPM
    RPM_M2 = -((posi_M2 - posiprev_M2) / 700.0) * (1 / deltaTime) * 60;
    posiprev_M1 = posi_M1;
    posiprev_M2 = posi_M2;
    prevT = currT;

    float maximum_limit = 251;
    InputM1 = map(RPM_M1, -maximum_limit, maximum_limit, 0, 510);
    InputM2 = map(RPM_M2, -maximum_limit, maximum_limit, 0, 510);
}

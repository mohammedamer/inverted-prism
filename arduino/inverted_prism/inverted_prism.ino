/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
#include <Wire.h>
#include <PID_v1.h>

// Motor Shield Rev3 – Channel A pins
const int dirA = 12;   // direction
const int brakeA = 9;  // brake
const int pwmA = 3;    // speed (PWM)

const int MPU = 0x68;  // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;

float AccErrorX = 0.65, AccErrorY = 4.2, GyroErrorX = 2.14, GyroErrorY = -3.01, GyroErrorZ = 0.05;

unsigned long previousTime = 0, currentTime = 0;
float elapsedTime = 0;
int c = 0;

bool initAngle = true;

double Setpoint;  // what you want (target angle, speed, etc.)
double Input;     // what you measure (current angle from IMU)
double Output;    // what PID computes (motor command)

// PID gains – start small and tune later
double Kp = 2.0;
double Ki = 0.0;
double Kd = 0.5;

double outLimit = 100;

double prevCmd = 0;

// Kp, Ki, Kd are your gains
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Wire.begin();                 // Initialize comunication
  Wire.beginTransmission(MPU);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);             // Talk to the register 6B
  Wire.write(0x00);             // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   //end the transmission

  // Call this function if you need to get the IMU error values for your module
  // calculate_IMU_error();

  // init motor
  pinMode(dirA, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(pwmA, OUTPUT);

  // Start with brake released
  digitalWrite(brakeA, LOW);

  Setpoint = 0.0;  // e.g. keep platform level at 0 degrees
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-outLimit, outLimit);  // so we can map to motor speed and direction

  delay(20);
}

void loop() {
  //=== Read acceleromter data === //

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers

  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis value

  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + AccErrorX;       // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + AccErrorY;  // AccErrorY ~(-1.58)

  if (initAngle) {
    gyroAngleX = accAngleX;
    gyroAngleY = accAngleY;

    initAngle = false;
  }

  // === Read gyroscope data === //
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;  // seconds
  previousTime = currentTime;                           // Divide by 1000 to get seconds

  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);  // Read 4 registers total, each axis value is stored in 2 registers

  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Correct the outputs with the calculated error values
  GyroX = GyroX + GyroErrorX;  // GyroErrorX ~(-0.56)
  GyroY = GyroY + GyroErrorY;  // GyroErrorY ~(2)
  GyroZ = GyroZ + GyroErrorZ;  // GyroErrorZ ~ (-0.8)

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime;  // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw = yaw + GyroZ * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Print the values on the serial monitor
  // Serial.print("roll: ");
  // Serial.println(roll);
  // Serial.print(" / pitch: ");
  // Serial.print(pitch);
  // Serial.print(" / yaw: ");
  // Serial.println(yaw);

  Input = -roll;

  // 2) Compute PID
  myPID.Compute();  // updates Output

  driveMotor(Output);


  // Optional debugging
  Serial.print("roll: ");
  Serial.print(roll);
  Serial.print(" / Output: ");
  Serial.println(Output);
}

// Map PID Output (-255..255) to motor direction + PWM
void driveMotor(double cmd) {
  // Deadband to avoid jitter near 0
  // if (cmd > -5 && cmd < 5) {
  //   analogWrite(pwmA, 0);
  //   return;
  // }

  if (cmd == 0) {
    digitalWrite(brakeA, HIGH);
    analogWrite(pwmA, 0);
    return;
  }

  if (cmd > 0) {
    
    if (prevCmd < 0){
      digitalWrite(brakeA, HIGH);
      analogWrite(pwmA, 0);

      delay(500);

      digitalWrite(brakeA, LOW);
    }

    digitalWrite(dirA, HIGH);     // one direction
    analogWrite(pwmA, (int)cmd);  // speed
  } else {


    if (prevCmd > 0){
      digitalWrite(brakeA, HIGH);
      analogWrite(pwmA, 0);

      delay(500);

      digitalWrite(brakeA, LOW);
    }

    digitalWrite(dirA, LOW);         // opposite direction
    analogWrite(pwmA, (int)(-cmd));  // speed is magnitude

    prevCmd = cmd;
  }
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
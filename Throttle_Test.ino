#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>

MPU6050 mpu;

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
float pitch, roll, yaw, lastTime, currentTime, deltaTime;
float gyroXOffset, gyroYOffset, gyroZOffset;

int throttlePin = 3;
int pitchPin = 2;
int rollPin = 18;
int yawPin = 19;

int motor_FL = 9;
int motor_FR = 10;
int motor_BR = 11;
int motor_BL = 12;

volatile unsigned long throttleStart, throttleEnd, pitchStart, pitchEnd, rollStart, rollEnd, yawStart, yawEnd;
volatile int throttlePulse, pitchPulse, rollPulse, yawPulse;

// Define EEPROM addresses for gyro offsets
const int EEPROM_ADDRESS_GYRO_X_OFFSET = 0;
const int EEPROM_ADDRESS_GYRO_Y_OFFSET = sizeof(float);
const int EEPROM_ADDRESS_GYRO_Z_OFFSET = 2 * sizeof(float);

Servo esc, esc2, esc3, esc4;

// PID variables
float setPoint_pitch = 0.0; // Desired pitch angle
float setPoint_roll = 0.0;  // Desired roll angle
int output_pitch = 0;
int output_roll = 0;
float integral_pitch = 0.0;
float integral_roll = 0.0;
float lastError_pitch = 0.0;
float lastError_roll = 0.0;

float Kp_pitch; // Proportional gain for pitch
float Ki_pitch; // Integral gain for pitch
float Kd_pitch; // Derivative gain for pitch

float Kp_roll;  // Proportional gain for roll
float Ki_roll;  // Integral gain for roll
float Kd_roll;  // Derivative gain for roll

float alpha = 0.02; // Complementary filter constant

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!loadCalibrationValues()) {
    calibrateGyro();
    saveCalibrationValues();
  }

  lastTime = millis() / 1000.0; // Start time in seconds

  pinMode(throttlePin, INPUT);
  pinMode(pitchPin, INPUT);
  pinMode(rollPin, INPUT);
  pinMode(yawPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pitchPin), pitchInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rollPin), rollInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(yawPin), yawInterrupt, CHANGE);

  esc.attach(motor_FL, 1000, 2000);
  esc2.attach(motor_FR, 1000, 2000);
  esc3.attach(motor_BL, 1000, 2000);
  esc4.attach(motor_BR, 1000, 2000);

  calibrateESC();

  // Ensure all ESCs are at the minimum throttle
  esc.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
}

void loop() {
  int throttle, pitch, roll, yaw;

  currentTime = millis() / 1000.0;
  deltaTime = currentTime - lastTime;
  lastTime = currentTime;

  // Read accelerometer and gyroscope data
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  // Apply offsets to gyroscope data
  gyroX -= gyroXOffset;
  gyroY -= gyroYOffset;
  gyroZ -= gyroZOffset;

  float accelX = accX / 16384.0;
  float accelY = accY / 16384.0;
  float accelZ = accZ / 16384.0;
  float gyroXrate = gyroX / 131.0;
  float gyroYrate = gyroY / 131.0;
  float gyroZrate = gyroZ / 131.0;

  // Calculate pitch and roll from accelerometer data
  float accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
  float accelRoll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

  // Complementary filter for pitch
  pitch = alpha * (pitch + gyroXrate * deltaTime) + (1 - alpha) * accelPitch;
  // Complementary filter for roll
  roll = alpha * (roll + gyroYrate * deltaTime) + (1 - alpha) * accelRoll;

  // Disable interrupts while reading shared variables
  noInterrupts();
  throttle = throttlePulse;
//  pitch = pitchPulse;
//  roll = rollPulse;
//  yaw = yawPulse;
  interrupts();

  // Limit throttle input to leave room for PID adjustments
  int maxThrottle = 1600; // Calculated throttle limit
  throttle = constrain(throttle, 1000, maxThrottle);

  Kp_pitch = 2.6; // Proportional gain for pitch
  Ki_pitch = 0.6; // Integral gain for pitch
  Kd_pitch = 1.2; // Derivative gain for pitch 
  
  Kp_roll = Kp_pitch;  // Proportional gain for roll
  Ki_roll = Ki_pitch;  // Integral gain for roll
  Kd_roll = Kd_pitch;  // Derivative gain for roll

  // PID Control for pitch
  float error_pitch = setPoint_pitch - pitch;
  integral_pitch += error_pitch * deltaTime;
  float derivative_pitch = (error_pitch - lastError_pitch) / deltaTime;
  integral_pitch = constrain(integral_pitch, -100, 100); // Limit integral windup
  error_pitch = constrain(error_pitch, -100, 100); // Limit integral windup
  derivative_pitch = constrain(derivative_pitch, -100, 100); // Limit integral windup
  output_pitch = (Kp_pitch * error_pitch) + (Ki_pitch * integral_pitch) + (Kd_pitch * derivative_pitch);
  lastError_pitch = error_pitch;

  // PID Control for roll
  float error_roll = setPoint_roll - roll;
  float derivative_roll = (error_roll - lastError_roll) / deltaTime;
  integral_roll += error_roll * deltaTime;
  integral_roll = constrain(integral_roll, -100, 100); // Limit integral windup
  error_roll = constrain(error_roll, -100, 100); // Limit integral windup
  derivative_roll = constrain(derivative_roll , -100, 100); // Limit integral windup
  output_roll = (Kp_roll * error_roll) + (Ki_roll * integral_roll) + (Kd_roll * derivative_roll);
  lastError_roll = error_roll;

  output_roll = constrain(output_roll, -400, 400);
  output_pitch = constrain(output_pitch, -400, 400);

  // Adjust throttle for each motor based on PID outputs
  int motor_FL_throttle = throttle + output_roll - output_pitch;
  int motor_FR_throttle = throttle + output_roll + output_pitch;
  int motor_BL_throttle = throttle - output_roll - output_pitch;
  int motor_BR_throttle = throttle - output_roll + output_pitch;

  // Constrain throttle values to be within the range
  motor_FL_throttle = constrain(motor_FL_throttle, 1050, 2000);
  motor_FR_throttle = constrain(motor_FR_throttle, 1050, 2000);
  motor_BL_throttle = constrain(motor_BL_throttle, 1050, 2000);
  motor_BR_throttle = constrain(motor_BR_throttle, 1050, 2000);

  // Use throttle value to control all motors
  if (pitchPulse <= 1400 || pitchPulse >= 1600 || rollPulse <= 1400 || rollPulse >= 1600) {
    esc.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    
  } else if (throttlePulse < 1100) {
    esc.writeMicroseconds(1100);
    esc2.writeMicroseconds(1100);
    esc3.writeMicroseconds(1100);
    esc4.writeMicroseconds(1100);
  } else {
    esc.writeMicroseconds(motor_FL_throttle);
    esc2.writeMicroseconds(motor_FR_throttle);
    esc3.writeMicroseconds(motor_BL_throttle);
    esc4.writeMicroseconds(motor_BR_throttle);
  }

  // Print pitch and roll angles and PID outputs to the Serial Monitor
  Serial.print("Pitch Angle: ");
  Serial.print((int)pitch); // Cast to int for display
  Serial.print("\tPitch Output: ");
  Serial.print(output_pitch);
  Serial.print("\tRoll Angle: ");
  Serial.print((int)roll); // Cast to int for display
  Serial.print("\tRoll Output: ");
  Serial.println(output_roll);
}

void calibrateESC() {
  // Calibrate ESC
  esc.writeMicroseconds(2000); // Max throttle
  esc2.writeMicroseconds(2000); // Max throttle
  esc3.writeMicroseconds(2000); // Max throttle
  esc4.writeMicroseconds(2000); // Max throttle
  delay(2000);

  esc.writeMicroseconds(1000); // Min throttle
  esc2.writeMicroseconds(1000); // Min throttle
  esc3.writeMicroseconds(1000); // Min throttle
  esc4.writeMicroseconds(1000); // Min throttle
  delay(2000);
}

void throttleInterrupt() {
  if (digitalRead(throttlePin) == HIGH) {
    throttleStart = micros();
  } else {
    throttleEnd = micros();
    throttlePulse = throttleEnd - throttleStart;
  }
}

void pitchInterrupt() {
  if (digitalRead(pitchPin) == HIGH) {
    pitchStart = micros();
  } else {
    pitchEnd = micros();
    pitchPulse = pitchEnd - pitchStart;
  }
}

void rollInterrupt() {
  if (digitalRead(rollPin) == HIGH) {
    rollStart = micros();
  } else {
    rollEnd = micros();
    rollPulse = rollEnd - rollStart;
  }
}

void yawInterrupt() {
  if (digitalRead(yawPin) == HIGH) {
    yawStart = micros();
  } else {
    yawEnd = micros();
    yawPulse = yawEnd - yawStart;
  }
}

void calibrateGyro() {
  const int numReadings = 2000;
  long gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;

  for (int i = 0; i < numReadings; i++) {
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
    gyroXSum += gyroX;
    gyroYSum += gyroY;
    gyroZSum += gyroZ;
  }

  gyroXOffset = gyroXSum / numReadings;
  gyroYOffset = gyroYSum / numReadings;
  gyroZOffset = gyroZSum / numReadings;
}

bool loadCalibrationValues() {
  EEPROM.get(EEPROM_ADDRESS_GYRO_X_OFFSET, gyroXOffset);
  EEPROM.get(EEPROM_ADDRESS_GYRO_Y_OFFSET, gyroYOffset);
  EEPROM.get(EEPROM_ADDRESS_GYRO_Z_OFFSET, gyroZOffset);
  return !(isnan(gyroXOffset) || isnan(gyroYOffset) || isnan(gyroZOffset));
}

void saveCalibrationValues() {
  EEPROM.put(EEPROM_ADDRESS_GYRO_X_OFFSET, gyroXOffset);
  EEPROM.put(EEPROM_ADDRESS_GYRO_Y_OFFSET, gyroYOffset);
  EEPROM.put(EEPROM_ADDRESS_GYRO_Z_OFFSET, gyroZOffset);
}

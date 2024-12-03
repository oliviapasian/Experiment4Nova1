/*********************************************************************
 * DF Pong Controller: NOVA.COMPONENT_1
 * Olivia Pasian
 * 
 * This program implements a Bluetooth Low Energy controller for Pong.
 * It sends movement data to a central device running in the browser and
 * provides audio feedback through a buzzer.
 *
 * It also uses the IMU sensor reading and orientation tracking with the
 * SensorFusion library to transform the raw values into degrees.
 *
 * Game Link : https://digitalfuturesocadu.github.io/df-pong/
 *
 * Code references:
 * https://github.com/DigitalFuturesOCADU/CC2024/tree/main/experiment4/Arduino/BLE/DFpong_controller_2button
 * https://github.com/DigitalFuturesOCADU/CC2024/tree/main/experiment4/Arduino/Sensors/IMU/imu_orientationData
 * 
 * Movement Values:
 * 0 = No movement / Neutral position
 * 1 = UP movement (paddle moves up)
 * 2 = DOWN movement (paddle moves down)
 * 3 = Handshake signal (used for initial connection verification)
 * 
 * Key Functions:
 * - readIMU(): Reading IMU data
 * - handleInput(): Process the IMU input to generate the states
 * - sendMovement(): Sends movement data over BLE (0-3)
 * - updateBLE(): Handles BLE connection management and updates
 * - updateBuzzer(): Provides different buzzer patterns for different movements
 * 
 * Key Variables:
 * - currentMovement: Stores current movement state (0-2)
 * - LED_PIN : Status of the arduino through the LED. 

 * Inputs:
 *   - Accelerometer (x, y, z) in G forces
 *   - Gyroscope (x, y, z) in degrees/second
 * 
 * Outputs:
 *   - Roll (x rotation) in degrees
 *   - Pitch (y rotation) in degrees 
 *   - Yaw (z rotation) in degrees
 * 
 **********************************************************/

#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include "SensorFusion.h"

//other code files
#include "ble_functions.h"
#include "buzzer_functions.h"

SF fusion;

//Controller name
const char* deviceName = "NOVA.COMPONENT_1";

// Pin definitions buzzer/LED
const int BUZZER_PIN = 11;       // Pin for haptic feedback buzzer
const int LED_PIN = LED_BUILTIN; // Status LED pin

// Movement state tracking
int currentMovement = 0;         // Current movement value (0=none, 1=up, 2=down, 3=handshake)

// Global variables for IMU
float gx, gy, gz, ax, ay, az;
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
float deltat = 0.0f;
unsigned long lastImuReadTime = 0;
unsigned int imuReadInterval = 10;  // Time between reads in milliseconds (100Hz)

// Calibration variables
float gyro_bias[3] = {0, 0, 0};  // Gyro bias in x, y, z
const int calibration_samples = 500;

void setup() 
{
  Serial.begin(9600);
  
  Serial.println("IMU Orientation test!");
  
  if (!initializeImu()) {
    Serial.println("IMU initialization failed!");
    while (1);
  }
  Serial.println("IMU initialized and calibrated!");

  // Configure LED for connection status indication
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize Bluetooth Low Energy with device name and status LED
  setupBLE(deviceName, LED_PIN);
  
  // Initialize buzzer for feedback
  setupBuzzer(BUZZER_PIN);
}


// Function to calibrate the IMU
void calibrateImu() {
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  int valid_samples = 0;
  
  Serial.println("Keep the IMU still for calibration...");
  delay(2000);  // Give user time to place IMU still
  
  Serial.println("Calibrating...");
  
  // Collect samples
  while (valid_samples < calibration_samples) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      // Accumulate gyro readings
      sum_gx += gx;
      sum_gy += gy;
      sum_gz += gz;
      
      // Accumulate accel readings
      sum_ax += ax;
      sum_ay += ay;
      sum_az += az;
      
      valid_samples++;
      
      // Show progress every 100 samples
      if (valid_samples % 100 == 0) {
        Serial.print("Progress: ");
        Serial.print((valid_samples * 100) / calibration_samples);
        Serial.println("%");
      }
      
      delay(2);  // Small delay between readings
    }
  }
  
  // Calculate average gyro bias
  gyro_bias[0] = sum_gx / calibration_samples;
  gyro_bias[1] = sum_gy / calibration_samples;
  gyro_bias[2] = sum_gz / calibration_samples;
  
  // Calculate initial orientation from average accelerometer readings
  float initial_ax = sum_ax / calibration_samples;
  float initial_ay = sum_ay / calibration_samples;
  float initial_az = sum_az / calibration_samples;
  
  // Update fusion filter with initial values
  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(0, 0, 0, initial_ax, initial_ay, initial_az, deltat);
  
  Serial.println("Calibration complete!");
  Serial.println("Gyro bias values:");
  Serial.print("X: "); Serial.print(gyro_bias[0]);
  Serial.print(" Y: "); Serial.print(gyro_bias[1]);
  Serial.print(" Z: "); Serial.println(gyro_bias[2]);
}

// Function to initialize the IMU
bool initializeImu() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    return false;
  }
  
  // Perform calibration
  calibrateImu();
  return true;
}

// Function to read IMU and update orientation values
void readImu() {
  unsigned long currentTime = millis();
  if (currentTime - lastImuReadTime >= imuReadInterval) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      // Read acceleration and gyroscope data
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      // Remove bias from gyro readings
      gx -= gyro_bias[0];
      gy -= gyro_bias[1];
      gz -= gyro_bias[2];
      
      // Convert gyroscope readings from deg/s to rad/s
      gx *= DEG_TO_RAD;
      gy *= DEG_TO_RAD;
      gz *= DEG_TO_RAD;
      
      // Update the filter
      deltat = fusion.deltatUpdate();
      fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);
      
      // Get the angles
      pitch = fusion.getPitch();
      roll = fusion.getRoll();
      yaw = fusion.getYaw();
      
      // Print the values
      printImuValues();
  
    }
    lastImuReadTime = currentTime;

  }
}

void handleInput(){
  //read IMU data
  readImu();

  // Setting currentMovement using IMU thresholds
  if (yaw >= 200) {
    currentMovement = 1;         // UP movement
  } else if (yaw <= 160) {
    currentMovement = 2;         // DOWN movement
  } else {
    currentMovement = 0;         // No movement
  }

  //send those updates to currentMovement to the buzzer function
  updateBuzzer(currentMovement);
}

// Function to print IMU values
void printImuValues() {
  Serial.print("Roll: ");
  Serial.print(roll, 1);
  Serial.print("\tPitch: ");
  Serial.print(pitch, 1);
  Serial.print("\tYaw: ");
  Serial.println(yaw, 1);
}

void loop() 
{
  // Update BLE connection status and handle incoming data
  updateBLE();

  //send the movement state to P5  
  sendMovement(currentMovement);

  //make the correct noise
  updateBuzzer(currentMovement);
  
  // Read the IMU (function handles timing internally)
  readImu();
  
  // Use IMU data to set currentMovement to 0, 1, 2
  handleInput();
}

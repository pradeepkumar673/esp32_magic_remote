#include "BleMouse.h"    // Include the Bluetooth Mouse library
#include <Wire.h>
#include <MPU6050.h>

// Create a BluetoothMouse object
BleMouse bleMouse;

// MPU6050 object
MPU6050 gyro;

// Sensitivity for cursor movement
float sensitivity = 1.2;  // Reduced sensitivity for smoother control

// Variables for accelerometer readings and angles
float axx = 0;
float ayy = 0;
float azz = 0;
float roll = 0;  // Angle around X-axis
float pitch = 0; // Angle around Y-axis

// Kalman filter variables
float roll_est = 0, roll_err = 1;
float pitch_est = 0, pitch_err = 1;
float process_noise = 0.01; // Process noise
float measurement_noise = 0.1; // Measurement noise

// Offset for calibration
float ay_offset = 0.0;

// Button pin setup
const int leftButtonPin  = 19; // Left mouse button on GPIO 19
const int rightButtonPin = 18; // Right mouse button on GPIO 18

// Track button states
bool leftPressed = false;
bool rightPressed = false;

void setup() {
  Serial.begin(115200);

  Wire.begin();
  gyro.initialize();

  // Calibration
  int numSamples = 100;
  int16_t ax, ay, az;
  for (int i = 0; i < numSamples; i++) {
    gyro.getAcceleration(&ax, &ay, &az);
    ay_offset += ay / 16384.0;
    delay(10);
  }
  ay_offset /= numSamples;

  bleMouse.begin();
  Serial.println("Bluetooth Mouse is ready to pair");

  pinMode(leftButtonPin, INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);
}

float kalmanFilter(float measurement, float &estimate, float &error) {
  error += process_noise;
  float kalman_gain = error / (error + measurement_noise);
  estimate += kalman_gain * (measurement - estimate);
  error *= (1 - kalman_gain);
  return estimate;
}

void loop() {
  if (bleMouse.isConnected()) {
    int16_t ax, ay, az;
    gyro.getAcceleration(&ax, &ay, &az);

    axx = ax / 16384.0;
    ayy = (ay / 16384.0) - ay_offset;
    azz = az / 16384.0;

    float roll_means = atan2(ayy, sqrt(axx * axx + azz * azz)) * 180 / PI; 
    float pitch_means = atan2(-axx, sqrt(ayy * ayy + azz * azz)) * 180 / PI; 

    roll = kalmanFilter(roll_means, roll_est, roll_err);
    pitch = kalmanFilter(pitch_means, pitch_est, pitch_err);

    // Reduced sensitivity movement
    int deltaY = roll * sensitivity;
    int deltaX = pitch * sensitivity;

    // Bigger deadzone → more stable control
    if (abs(roll) < 4) deltaY = 0;    
    if (abs(pitch) < 4) deltaX = 0;   

    bleMouse.move(deltaX, -deltaY);

    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | DeltaX: "); Serial.print(deltaX);
    Serial.print(" | DeltaY: "); Serial.println(deltaY);

    // ----- Buttons -----
    bool leftState = (digitalRead(leftButtonPin) == LOW);
    bool rightState = (digitalRead(rightButtonPin) == LOW);

    if (leftState && !leftPressed) {
      bleMouse.press(MOUSE_LEFT);
      leftPressed = true;
      Serial.println("Left Button DOWN");
    } else if (!leftState && leftPressed) {
      bleMouse.release(MOUSE_LEFT);
      leftPressed = false;
      Serial.println("Left Button UP");
    }

    if (rightState && !rightPressed) {
      bleMouse.press(MOUSE_RIGHT);
      rightPressed = true;
      Serial.println("Right Button DOWN");
    } else if (!rightState && rightPressed) {
      bleMouse.release(MOUSE_RIGHT);
      rightPressed = false;
      Serial.println("Right Button UP");
    }

    delay(20);  
  } else {
    Serial.println("Mouse not connected");
    delay(1000);
  }
}

#include <Wire.h>
#include <MPU6050_light.h>
#include <Encoder.h>
#include<SoftwareSerial.h>

MPU6050 mpu(Wire);

// Motor Pins
#define inputpin1 A2    // Left motor
#define inputpin2 A3
#define inputpin3 9    // Right motor
#define inputpin4 4

#define EN1 6
#define EN2 5

// SoftwareSerial Pins for HC-05
#define RX_PIN A1
#define TX_PIN A0
SoftwareSerial bluetooth(TX_PIN, RX_PIN);

// Encoder Pins
Encoder enc(2, 3);

unsigned long lastTime = 0;
unsigned long timer = 0;
unsigned long prevTime = 0;
unsigned long positionLastTime = 0;

float motorSpeed = 0;
long old_ticks = 0;

// PID parameters for angle control
float Kp_angle = 110;  // 130
float Ki_angle = 1800;   // 1800 
float Kd_angle = 9.5;   // 7.5

// PID parameters for position control
float Kp_position = 130;  // 130
float Ki_position = 225.15;  // 95.15 
float Kd_position = 5; // 5

// Variables for angle PID
float tiltSetpoint_0 = 0;
float tiltSetpoint_f = 1;
float tiltSetpoint_b = -1;
float error_angle = 0;
float lastError_angle = 0;
float integral_angle = 0;
float output_angle = 0;
float derivative_angle = 0;

float pwm_0 = 0;
float pwm_f = 0;
float pwm_b = 0;

// Variables for position PID
long positionSetpoint = 0;
float lastError_position = 0;
float integral_position = 0;
float derivative_position = 0;
float error_position = 0;
float currentPos = 0;
float prevPos = 0;
float output_position = 0;

// MPU variables
float currentAngle = 0;
float currentAngle_new = 0;
float currentAngle_old = 0;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);

  Serial.println("Bluetooth is ready!");

  Wire.begin();
  mpu.begin();

  delay(2000);
  mpu.calcOffsets();
  
  // Initialize motor pins
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(inputpin1, OUTPUT);
  pinMode(inputpin2, OUTPUT);
  pinMode(inputpin3, OUTPUT);
  pinMode(inputpin4, OUTPUT);

  digitalWrite(inputpin1, LOW);
  digitalWrite(inputpin2, LOW);
  digitalWrite(inputpin3, LOW);
  digitalWrite(inputpin4, LOW);

  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
   
}

float encoder() {
  long ticks = enc.read();  // Get raw tick count
  
  // Only divide when using or displaying the value
  float reduced_ticks = ticks * (360.0/2800.0);

  float distance = reduced_ticks * (14.14 / 360.0);

  if (reduced_ticks != old_ticks) {
    old_ticks = reduced_ticks;
  }

  return distance;

}

float PID(float angleSetpoint) {
  unsigned long currentTime = millis();
  double dt = (currentTime - prevTime) / 1000.0; // Calculate time step in seconds
  prevTime = currentTime;

  mpu.update();
  double acclX = mpu.getAngleX();
  // Calculate angular velocity from gyroscope
  double gyroRate = mpu.getGyroX();  // Gyroscope X-axis rate (degrees/sec)
  currentAngle_new = 0.985 * (currentAngle_old + gyroRate * dt) + (1 - 0.985) * acclX;
  currentAngle_old = currentAngle_new;
  currentAngle = currentAngle_new;

  // position PID
  currentPos = encoder();
  error_position = positionSetpoint - currentPos;
  integral_position += error_position * dt;
  derivative_position = (error_position - lastError_position) / dt;
  integral_position = constrain(integral_position, -255, 255);
  output_position = Kp_position * error_position +
                    Ki_position * integral_position +
                    Kd_position * derivative_position;
  lastError_position = error_position;

  // Angle PID
  error_angle = angleSetpoint - currentAngle;
  integral_angle += error_angle * dt;
  derivative_angle = (error_angle - lastError_angle) / dt;
  integral_angle = constrain(integral_angle, -255, 255);  // Anti-windup
  output_angle = Kp_angle * error_angle +
                 Ki_angle * integral_angle + 
                 Kd_angle * derivative_angle;
  lastError_angle = error_angle;

  motorSpeed = constrain(output_position + output_angle, -170, 170);

  return motorSpeed;
}  

void forward(float pwm){

  if (pwm > 0) {
    digitalWrite(inputpin1, HIGH);
    digitalWrite(inputpin2, LOW);
    digitalWrite(inputpin3, LOW);
    digitalWrite(inputpin4, HIGH);
  } else {
    digitalWrite(inputpin1, LOW);
    digitalWrite(inputpin2, HIGH);
    digitalWrite(inputpin3, HIGH);
    digitalWrite(inputpin4, LOW);
  }
  analogWrite(EN1, abs(pwm));
  analogWrite(EN2, abs(pwm));
}

void backward(float pwm){

  if (pwm > 0) {
    digitalWrite(inputpin1, HIGH);
    digitalWrite(inputpin2, LOW);
    digitalWrite(inputpin3, LOW);
    digitalWrite(inputpin4, HIGH);
  } else {
    digitalWrite(inputpin1, LOW);
    digitalWrite(inputpin2, HIGH);
    digitalWrite(inputpin3, HIGH);
    digitalWrite(inputpin4, LOW);
  }
  analogWrite(EN1, abs(pwm));
  analogWrite(EN2, abs(pwm));

}

void left(float pwm) {
  if (pwm > 0) {
    digitalWrite(inputpin1, HIGH);
    digitalWrite(inputpin2, LOW);
    digitalWrite(inputpin3, LOW);
    digitalWrite(inputpin4, HIGH);
  } else {
    digitalWrite(inputpin1, LOW);
    digitalWrite(inputpin2, HIGH);
    digitalWrite(inputpin3, HIGH);
    digitalWrite(inputpin4, LOW);
  }
  analogWrite(EN1, abs(pwm) * 0.6);
  analogWrite(EN2, abs(pwm) + 68);
}

void right(float pwm) {
  if (pwm > 0) {
    digitalWrite(inputpin1, HIGH);
    digitalWrite(inputpin2, LOW);
    digitalWrite(inputpin3, LOW);
    digitalWrite(inputpin4, HIGH);
  } else {
    digitalWrite(inputpin1, LOW);
    digitalWrite(inputpin2, HIGH);
    digitalWrite(inputpin3, HIGH);
    digitalWrite(inputpin4, LOW);
  }
  analogWrite(EN1, abs(pwm) + 68);
  analogWrite(EN2, abs(pwm) * 0.6);
}

void loop() {
  if (millis() - timer >= 10){
    // Check for Bluetooth input
    if (bluetooth.available()) {
      char input = bluetooth.read();
      Serial.println(input);
      switch(input){
        
        // uppercase when key is pressed || lowercase when key is released (from bluetooth app)
        case 'F':
          pwm_f = PID(tiltSetpoint_f);
          forward(pwm_f);
          break;

        case 'B':
          pwm_b = PID(tiltSetpoint_b);
          backward(pwm_b);
          break;

        case 'R':
          pwm_0 = PID(tiltSetpoint_0);
          right(pwm_0);
          break;

        case 'L':
          pwm_0 = PID(tiltSetpoint_0);
          left(pwm_0);
          break;
        
        // when key is released, simply balance the bot at tiltSetpoint = 0
        case 'f':
        case 'b':
        case 'r':
        case 'l':
          break;

        default:
          break;
      }
    }
    else{
      pwm_0 = PID(tiltSetpoint_0);
      
      if (pwm_0 > 0) {
        digitalWrite(inputpin1, HIGH);
        digitalWrite(inputpin2, LOW);
        digitalWrite(inputpin3, LOW);
        digitalWrite(inputpin4, HIGH);
      } else {
        digitalWrite(inputpin1, LOW);
        digitalWrite(inputpin2, HIGH);
        digitalWrite(inputpin3, HIGH);
        digitalWrite(inputpin4, LOW);
      }
      analogWrite(EN1, abs(pwm_0));
      analogWrite(EN2, abs(pwm_0));
    }

    timer = millis();
  }
}

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <WiFi.h>
#include <Servo.h>

//seperate file on arduino sketch to remove personal information of hotspot details
#include "untitled.h"

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);


// Cloud boolean variable to control the system
bool _switch_ = false;


Servo thisservo;

// Pin Definitions
#define SERVO_PIN 9
#define IN1 6
#define IN2 7
#define IN3 4
#define IN4 5

// Servo Timing Constants (ms)
#define SERVO_RIGHT_DURATION 700
#define SERVO_PAUSE1_DURATION 450
#define SERVO_LEFT_DURATION 450
#define SERVO_PAUSE2_DURATION 250

// Motor Timing Constants (ms)
#define MOTOR_STOP_INITIAL 1000
#define MOTOR_FORWARD_DURATION 2000
#define MOTOR_TURN_DURATION 1000
#define MOTOR_PAUSE_AFTER_TURN 500

enum ServoState {
  SERVO_RIGHT,
  SERVO_CENTER_AFTER_RIGHT,
  SERVO_LEFT,
  SERVO_CENTER_AFTER_LEFT,
  SERVO_WAIT
};

enum MotorState {
  MOTOR_INITIAL_STOP,
  MOTOR_FORWARD,
  MOTOR_TURN,
  MOTOR_POST_TURN_STOP
};

unsigned long previousServoMillis = 0;
unsigned long previousMotorMillis = 0;

ServoState servoState = SERVO_RIGHT;
MotorState motorState = MOTOR_INITIAL_STOP;

bool turnRightNext = true;

// Link the cloud variable
void initProperties() {
  ArduinoCloud.addProperty(_switch_, READWRITE);
}

void setup() {
  Serial.begin(115200);

  thisservo.attach(SERVO_PIN);
  thisservo.write(90);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();
  delay(1000);

  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  initProperties();
}

void loop() {
  ArduinoCloud.update();

  // Only run motors and servo if _switch_ is true
  if (_switch_ == 1) {
    unsigned long currentMillis = millis();
    updateServo(currentMillis);
    updateMotors(currentMillis);
  } else {
    stopMotors();
    thisservo.write(90);  // Center servo when off
  }
}


void updateServo(unsigned long currentMillis) {
  if (servoState == SERVO_RIGHT) {
    thisservo.write(115);
    previousServoMillis = currentMillis;
    servoState = SERVO_CENTER_AFTER_RIGHT;
    return;
  }
  else if (servoState == SERVO_CENTER_AFTER_RIGHT) {
    if (currentMillis - previousServoMillis >= SERVO_RIGHT_DURATION) {
      thisservo.write(90);
      previousServoMillis = currentMillis;
      servoState = SERVO_LEFT;
    }
    return;
  }
  else if (servoState == SERVO_LEFT) {
    if (cunrrentMillis - previousServoMillis >= SERVO_PAUSE1_DURATION) {
      thisservo.write(70);
      previousServoMillis = currentMillis;
      servoState = SERVO_CENTER_AFTER_LEFT;
    }
    return;
  }
  else if (servoState == SERVO_CENTER_AFTER_LEFT) {
    if (currentMillis - previousServoMillis >= SERVO_LEFT_DURATION) {
      thisservo.write(90);
      previousServoMillis = currentMillis;
      servoState = SERVO_WAIT;
    }
    return;
  }
  else if (servoState == SERVO_WAIT) {
    if (currentMillis - previousServoMillis >= SERVO_PAUSE2_DURATION) {
      servoState = SERVO_RIGHT;  // Restart cycle
    }
    return;
  }
}

void updateMotors(unsigned long currentMillis) {
  if (motorState == MOTOR_INITIAL_STOP) {
    stopMotors();
    previousMotorMillis = currentMillis;
    motorState = MOTOR_FORWARD;
    return;
  }
  else if (motorState == MOTOR_FORWARD) {
    if (currentMillis - previousMotorMillis >= MOTOR_STOP_INITIAL) {
      moveForward();
      previousMotorMillis = currentMillis;
      motorState = MOTOR_TURN;
    }
    return;
  }
  else if (motorState == MOTOR_TURN) {
    if (currentMillis - previousMotorMillis >= MOTOR_FORWARD_DURATION) {
      if (turnRightNext) {
        turnRight();
      } else {
        turnLeft();
      }
      previousMotorMillis = currentMillis;
      motorState = MOTOR_POST_TURN_STOP;
    }
    return;
  }
  else if (motorState == MOTOR_POST_TURN_STOP) {
    if (currentMillis - previousMotorMillis >= MOTOR_TURN_DURATION) {
      stopMotors();
      previousMotorMillis = currentMillis;
      motorState = MOTOR_INITIAL_STOP;
      turnRightNext = !turnRightNext;  // Alternate turn direction
    }
    return;
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

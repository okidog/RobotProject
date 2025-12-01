#include <Motor.h>
#include <IRremote.h>

const byte leftMotorPin1 = 4;
const byte leftMotorPin2 = 5;
const byte leftMotorPin3 = 6;
const byte leftMotorPin4 = 7;
const byte rightMotorPin1 = A0;
const byte rightMotorPin2 = A1;
const byte rightMotorPin3 = A2;
const byte rightMotorPin4 = A3;
const byte ultrasonicTrigger = A4;
const byte ultrasonicEcho = A5;

const byte servoPin = 9;

const byte IR_RECIEVE_PIN = 3;

class Ultrasonic {
  private: 
  	uint8_t trigPin = A4;
  	uint8_t echoPin = A5;
  public:
  void setPins(uint8_t newTrigPin, uint8_t newEchoPin) {
    trigPin = newTrigPin;
    echoPin = newEchoPin;
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }
};

Ultrasonic ultrasonic;
Motor leftMotor = {leftMotorPin1, leftMotorPin2, leftMotorPin3, leftMotorPin4, false, 1};
Motor rightMotor = {rightMotorPin1, rightMotorPin2, rightMotorPin3, rightMotorPin4, true, 1}; 


void setup() {
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(leftMotorPin3, OUTPUT);
  pinMode(leftMotorPin4, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(rightMotorPin3, OUTPUT);
  pinMode(rightMotorPin4, OUTPUT);
  ultrasonic.setPins(ultrasonicTrigger, ultrasonicEcho);

}

void loop() {
  leftMotor.forward(2000, 1);
  rightMotor.forward(2000, 1);

}

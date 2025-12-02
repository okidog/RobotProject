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

int speed = 2000;

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
Motor leftMotor = {leftMotorPin1, leftMotorPin2, leftMotorPin3, leftMotorPin4, false, 2.f};
Motor rightMotor = {rightMotorPin1, rightMotorPin2, rightMotorPin3, rightMotorPin4, true, 2.f}; 


void setup() {
  Serial.begin(9600);

  ultrasonic.setPins(ultrasonicTrigger, ultrasonicEcho);

  leftMotor.forward(speed);
  rightMotor.forward(speed);
}

void loop() {
  leftMotor.runSpeedToPosition();
  rightMotor.runSpeedToPosition();
  Serial.print(leftMotor.distanceToGo());
  Serial.print('\n');
  Serial.print(rightMotor.distanceToGo());
  Serial.print('\n');
}

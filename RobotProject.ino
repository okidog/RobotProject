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


void setup() {
  pinMode(leftMotorPin1, INPUT);
  pinMode(leftMotorPin2, INPUT);
  pinMode(leftMotorPin3, INPUT);
  pinMode(leftMotorPin4, INPUT);
  pinMode(leftMotorPin1, INPUT);
  setPins(ultrasonicTrigger, ultrasonicEcho);

}

void loop() {
  // put your main code here, to run repeatedly:

}

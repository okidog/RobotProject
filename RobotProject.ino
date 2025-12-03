#include <IRremote.h>
#include <Servo.h>
#include "Motor.h"

// ----------------------- Pin Definitions -----------------------
const byte ultrasonicTrigger = A4;
const byte ultrasonicEcho    = A5;

const uint8_t leftMotorPin1 = 4;
const uint8_t leftMotorPin2 = 5;
const uint8_t leftMotorPin3 = 6;
const uint8_t leftMotorPin4 = 7;

const uint8_t rightMotorPin1 = A0;
const uint8_t rightMotorPin2 = A1;
const uint8_t rightMotorPin3 = A2;
const uint8_t rightMotorPin4 = A3;

const byte servoPin       = 9;
const byte IR_RECEIVE_PIN = 3;

int stepCount = 0;
int dist = 0;
int acceleration = 9;
const int stepSize = 2000;
const int turnLoopSize = 30000;

int speed = 600;
// ----------------------- Ultrasonic Class -----------------------
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

    long readCM() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      long duration = pulseIn(echoPin, HIGH, 30000); 
      if (duration == 0) return 400;

      return duration * 0.0343 / 2;
    }

    int getDistanceCM() {
      return (int)readCM();
    }
};

Ultrasonic ultrasonic;

Motor leftMotor = {leftMotorPin1, leftMotorPin2, leftMotorPin3, leftMotorPin4, false, 2.f};
Motor rightMotor = {rightMotorPin1, rightMotorPin2, rightMotorPin3, rightMotorPin4, true, 2.f};

// ----------------------- Servo -----------------------
Servo scannerServo;
const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 160;   // wide scan
const int SERVO_RIGHT  = 20;

// ----------------------- IR Remote -----------------------
IRrecv irReceiver(IR_RECEIVE_PIN);
decode_results results;

// Button codes
const unsigned long IR_UP    = 0xFF629D;
const unsigned long IR_DOWN  = 0xFFA857;
const unsigned long IR_LEFT  = 0xFF22DD;
const unsigned long IR_RIGHT = 0xFFC23D;

// ----------------------- State Machine -----------------------
enum RobotState {
  STATE_FORWARD,
  STATE_SCAN,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT
};

RobotState currentState = STATE_FORWARD;
unsigned long stateStartTime = 0;

// Thresholds
const int WALL_DISTANCE = 10; 
const int SCAN_TIME     = 300;
const int TURN_TIME     = 500;

bool moving = false;

int turnDirection = 1; // +1 = left, -1 = right

// ----------------------- State Helpers -----------------------
void changeState(RobotState newState) {
  currentState = newState;
  stateStartTime = millis();

  switch (newState) {
    case STATE_FORWARD:     Serial.println("STATE: FORWARD");     break;
    case STATE_SCAN:        Serial.println("STATE: SCAN");        break;
    case STATE_TURN_LEFT:   Serial.println("STATE: TURN LEFT");   break;
    case STATE_TURN_RIGHT:  Serial.println("STATE: TURN RIGHT");  break;
  }
}

void performScanAndChooseTurn() {
  Serial.println("SCANNING LEFT/RIGHT...");

  // LEFT
  scannerServo.write(SERVO_LEFT);
  delay(300);
  int leftDist = ultrasonic.getDistanceCM();
  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.println(" cm");

  // RIGHT
  scannerServo.write(SERVO_RIGHT);
  delay(300);
  int rightDist = ultrasonic.getDistanceCM();
  Serial.print("Right: ");
  Serial.print(rightDist);
  Serial.println(" cm");

  // CENTER
  scannerServo.write(SERVO_CENTER);
  delay(200);

  // Choose turn direction
  if (leftDist < WALL_DISTANCE && rightDist < WALL_DISTANCE) {
    Serial.println("I haven't accounted for this yet dear god help us.");
  } else 
    if (leftDist > rightDist) {
      Serial.println("Decision: TURN LEFT");
      changeState(STATE_TURN_LEFT);
      moving = false;
    } else {
      Serial.println("Decision: TURN RIGHT");
      changeState(STATE_TURN_RIGHT);
      moving = false;
    }
}

void setMotorTargets() {
  leftMotor.setCurrentPosition(0);
  rightMotor.setCurrentPosition(0);

  switch (currentState) {
    case STATE_TURN_LEFT:
      leftMotor.moveTo(-stepSize);
      rightMotor.moveTo(stepSize);
      Serial.println("LEFT TARGETS INITD");
      break;
    case STATE_TURN_RIGHT:
      leftMotor.moveTo(-stepSize);
      rightMotor.moveTo(-stepSize);
      Serial.println("RIGHT TARGETS INITD");
      break;
    default:
      leftMotor.moveTo(stepSize);
      rightMotor.moveTo(-stepSize);
      Serial.println("FORWARD TARGETS INITD");
      break;
  }
  leftMotor.setSpeed(speed);
  rightMotor.setSpeed(speed);
}

// ----------------------- Setup -----------------------
void setup() {
  Serial.begin(9600);

  ultrasonic.setPins(ultrasonicTrigger, ultrasonicEcho);

  scannerServo.attach(servoPin);
  scannerServo.write(SERVO_CENTER);

  irReceiver.enableIRIn();

  changeState(STATE_FORWARD);

  /*leftMotor.setSpeed(speed);
  rightMotor.setSpeed(speed);

  leftMotor.setAcceleration(acceleration);
  rightMotor.setAcceleration(acceleration); */
}

// ----------------------- Loop -----------------------
void loop() {

  // ---------- IR REMOTE ----------
  if (irReceiver.decode(&results)) {
    unsigned long code = results.value;

    Serial.print("IR Code: 0x");
    Serial.println(code, HEX);

    if (code == IR_UP)    Serial.println("IR: UP (nudge fwd placeholder)");
    if (code == IR_DOWN)  Serial.println("IR: DOWN (nudge back placeholder)");
    if (code == IR_LEFT)  Serial.println("IR: LEFT (nudge left placeholder)");
    if (code == IR_RIGHT) Serial.println("IR: RIGHT (nudge right placeholder)");

    irReceiver.resume();
  }

  // ---------- STATE MACHINE ----------

  if (!moving) {
    switch (currentState) {

      case STATE_FORWARD:
        moving = true;
        break;

      case STATE_SCAN:
        performScanAndChooseTurn();
        moving = false;
        break;

      case STATE_TURN_LEFT:
        Serial.println("turn left state reached");
        Serial.println(turnLoopSize);
        setMotorTargets();
        for (int i = 0; i < turnLoopSize; i++) {
          leftMotor.runSpeedToPosition();
          rightMotor.runSpeedToPosition();

          if (abs(leftMotor.distanceToGo()) <= 0) {
            Serial.println("reset triggd");
            setMotorTargets();
          }
        }
        moving = true;
        Serial.println("turn left state exiting ... ");
        changeState(STATE_FORWARD);
        break;

      case STATE_TURN_RIGHT:
        Serial.println("turn right state reached");
        setMotorTargets();
        for (int i = 0; i < turnLoopSize; i++) {
          leftMotor.runSpeedToPosition();
          rightMotor.runSpeedToPosition();

          if (abs(leftMotor.distanceToGo()) <= 0) {
            Serial.println("reset triggd");
            setMotorTargets();
          }
        }
        moving = true;
        Serial.println("turn left state exiting ... ");
        changeState(STATE_FORWARD);
        break;
    }
  }

  if (moving) {
    leftMotor.runSpeedToPosition();
    rightMotor.runSpeedToPosition();

    if (abs(leftMotor.distanceToGo()) <= 0) {
      Serial.println("reset triggd");
      setMotorTargets();
    } 
    stepCount++; 

    if (stepCount >= 10000) {

      dist = ultrasonic.getDistanceCM();

      Serial.print("Front Distance: ");
      Serial.print(dist);
      Serial.println(" cm");

      if (dist < WALL_DISTANCE) {
        changeState(STATE_SCAN);
        moving = false;
      }
      
      stepCount = 0;
    }
  }
}

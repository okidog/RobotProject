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
const long int turnLoopSize = 30000;

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

Motor leftMotor  = {leftMotorPin1, leftMotorPin2, leftMotorPin3, leftMotorPin4, false, 2.f};
Motor rightMotor = {rightMotorPin1, rightMotorPin2, rightMotorPin3, rightMotorPin4, true, 2.f};

// ----------------------- Servo -----------------------
Servo scannerServo;
const int SERVO_CENTER = 97;
const int SERVO_LEFT   = 180;   // wide scan
const int SERVO_RIGHT  = 0;

// ----------------------- IR Remote -----------------------
IRrecv irReceiver(IR_RECEIVE_PIN);
decode_results results;

// Button codes (your known ones)
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
const int WALL_DISTANCE = 15; 
const int SCAN_TIME     = 300;
const int TURN_TIME     = 500;

bool moving = false;
int turnDirection = 1; // +1 = left, -1 = right

// ----------------------- IR RC-style control flags -----------------------
const unsigned long IR_DEADBAND = 300;  // stop motors after release
const unsigned long IR_RETURN   = 600;  // return to auto after release

unsigned long lastIRtime = 0;
bool manualActive = false;

// ----------------------- Helpers -----------------------
void changeState(RobotState newState) {
  currentState = newState;
  stateStartTime = millis();

  switch (newState) {
    case STATE_FORWARD:   Serial.println("STATE: FORWARD");   break;
    case STATE_SCAN:      Serial.println("STATE: SCAN");      break;
    case STATE_TURN_LEFT: Serial.println("STATE: TURN LEFT"); break;
    case STATE_TURN_RIGHT:Serial.println("STATE: TURN RIGHT");break;
  }
}

void performScanAndChooseTurn() {
  Serial.println("SCANNING LEFT/RIGHT...");

  // LEFT
  scannerServo.write(SERVO_LEFT);
  delay(1000);
  int leftDist = ultrasonic.getDistanceCM();
  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.println(" cm");

  // RIGHT
  scannerServo.write(SERVO_RIGHT);
  delay(1000);
  int rightDist = ultrasonic.getDistanceCM();
  Serial.print("Right: ");
  Serial.print(rightDist);
  Serial.println(" cm");

  // CENTER
  scannerServo.write(SERVO_CENTER);
  delay(400);

  // Choose turn direction
  if (leftDist < WALL_DISTANCE && rightDist < WALL_DISTANCE) {
    Serial.println("I haven't accounted for this yet dear god help us.");
  } else if (leftDist > rightDist) {
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
      rightMotor.moveTo(-stepSize);
      Serial.println("LEFT TARGETS INITD");
      break;
    case STATE_TURN_RIGHT:
      leftMotor.moveTo(stepSize);
      rightMotor.moveTo(stepSize);
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

// ----------------------- IR PRESS-AND-HOLD HANDLER -----------------------
void handleIR() {
  if (irReceiver.decode(&results)) {
    unsigned long code = results.value;

    Serial.print("IR Code: 0x");
    Serial.println(code, HEX);

    // Only treat our 4 buttons as manual control
    if (code == IR_UP || code == IR_DOWN || code == IR_LEFT || code == IR_RIGHT) {
      manualActive = true;
      lastIRtime = millis();

      // RC-style: every press/hold nudges motion in that direction
      leftMotor.setCurrentPosition(0);
      rightMotor.setCurrentPosition(0);

      if (code == IR_UP) {
        // forward
        leftMotor.move(stepSize);
        rightMotor.move(-stepSize);
      } else if (code == IR_DOWN) {
        // backward
        leftMotor.move(-stepSize);
        rightMotor.move(stepSize);
      } else if (code == IR_LEFT) {
        // spin left
        leftMotor.move(-stepSize);
        rightMotor.move(-stepSize);
      } else if (code == IR_RIGHT) {
        // spin right
        leftMotor.move(stepSize);
        rightMotor.move(stepSize);
      }

      leftMotor.setSpeed(speed);
      rightMotor.setSpeed(speed);
      moving = true;
    }

    irReceiver.resume();
  }

  if (manualActive) {
    unsigned long elapsed = millis() - lastIRtime;

    // After short gap with no new IR: stop motors, but stay in manual mode
    if (elapsed > IR_DEADBAND && elapsed <= IR_RETURN) {
      leftMotor.stop();
      rightMotor.stop();
      moving = false;
    }

    // After longer gap: return to auto mode
    if (elapsed > IR_RETURN) {
      manualActive = false;
      Serial.println("IR idle → back to AUTO");
      changeState(STATE_FORWARD);
      moving = false; // let auto state machine re-arm
    }
  }
}

// ----------------------- Setup -----------------------
void setup() {
  Serial.begin(9600);

  ultrasonic.setPins(ultrasonicTrigger, ultrasonicEcho);

  scannerServo.attach(servoPin);
  scannerServo.write(SERVO_CENTER);

  irReceiver.enableIRIn();

  changeState(STATE_FORWARD);
}

// ----------------------- Loop -----------------------
void loop() {

  // ---------- IR PRESS-AND-HOLD CONTROL ----------
  handleIR();

  // If we're manually overriding, just drive like RC and skip auto logic
  if (manualActive) {
    if (moving) {
      leftMotor.runSpeedToPosition();
      rightMotor.runSpeedToPosition();
    }
    return;  // don't run auto navigation while in manual
  }

  // ---------- STATE MACHINE (AUTONOMOUS) ----------
  if (!moving) {
    switch (currentState) {

      case STATE_FORWARD:
        moving = true;
        break;

      case STATE_SCAN:
        delay(1000);
        performScanAndChooseTurn();
        moving = false;
        break;

      case STATE_TURN_LEFT:
        Serial.println("turn left state reached");
        Serial.println(turnLoopSize);
        setMotorTargets();
        for (long int i = 0; i < turnLoopSize; i++) {
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
        setMotorTargets();
        break;

      case STATE_TURN_RIGHT:
        Serial.println("turn right state reached");
        setMotorTargets();
        for (long int i = 0; i < turnLoopSize; i++) {
          leftMotor.runSpeedToPosition();
          rightMotor.runSpeedToPosition();

          if (abs(leftMotor.distanceToGo()) <= 0) {
            Serial.println("reset triggd");
            setMotorTargets();
          }
        }
        moving = true;
        Serial.println("turn right state exiting ... ");
        changeState(STATE_FORWARD);
        setMotorTargets();
        break;
    }
  }

  // ---------- MOTOR EXECUTION (AUTONOMOUS) ----------
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

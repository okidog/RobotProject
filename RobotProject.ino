#include <Motor.h>
#include <IRremote.h>

// ----------------------- Pin Definitions -----------------------
const byte leftMotorPin1 = 4;
const byte leftMotorPin2 = 5;
const byte leftMotorPin3 = 6;
const byte leftMotorPin4 = 7;

const byte rightMotorPin1 = A0;
const byte rightMotorPin2 = A1;
const byte rightMotorPin3 = A2;
const byte rightMotorPin4 = A3;

const byte ultrasonicTrigger = A4;
const byte ultrasonicEcho    = A5;

const byte servoPin = 9;

const byte IR_RECEIVE_PIN = 3;

int speed = 500;
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
  Serial.print('\n');
    }

    // placeholder for now
    int getDistanceCM() {
      return random(10, 60);
    }
};

Ultrasonic ultrasonic;
Motor leftMotor = {leftMotorPin1, leftMotorPin2, leftMotorPin3, leftMotorPin4, false, 2.f};
Motor rightMotor = {rightMotorPin1, rightMotorPin2, rightMotorPin3, rightMotorPin4, true, 2.f}; 

// ----------------------- State Machine -----------------------
enum RobotState {
  STATE_FORWARD,
  STATE_SCAN,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT
};

RobotState currentState = STATE_FORWARD;

unsigned long stateStartTime = 0;

bool moving = false;

// for now
const int WALL_DISTANCE = -1;  // cm 
const int SCAN_TIME = 300;     // ms
const int TURN_TIME = 500;     // ms


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

// ----------------------- Setup -----------------------
IRrecv irReceiver(IR_RECEIVE_PIN);
decode_results results;

void setup() {
  Serial.begin(9600);

  ultrasonic.setPins(ultrasonicTrigger, ultrasonicEcho);

  irReceiver.enableIRIn();  // Start IR receiver

  changeState(STATE_FORWARD);
  ultrasonic.setPins(ultrasonicTrigger, ultrasonicEcho);

  leftMotor.moveTo(5000);
  rightMotor.moveTo(-5000);

  leftMotor.setSpeed(speed);
  rightMotor.setSpeed(speed);
}

// ----------------------- Loop -----------------------
void loop() {

  // ---------------- IR REMOTE CHECK ----------------
  if (irReceiver.decode(&results)) {
    unsigned long code = results.value;

    Serial.print("IR Code: 0x");
    Serial.println(code, HEX);

    switch (code) {
      case 0xFF629D:  // UP
        Serial.println("IR COMMAND: FORWARD");
        break;

      case 0xFFA857:  // DOWN
        Serial.println("IR COMMAND: BACKWARD");
        break;

      case 0xFF22DD:  // LEFT
        Serial.println("IR COMMAND: LEFT");
        break;

      case 0xFFC23D:  // RIGHT
        Serial.println("IR COMMAND: RIGHT");
        break;

      default:
        Serial.println("UNKNOWN IR BUTTON");
        break;
    }

    irReceiver.resume(); // Ready for next IR value
  }

  // ---------------- STATE MACHINE ----------------
  int dist = ultrasonic.getDistanceCM();

  if (!moving) {
    switch (currentState) {

      case STATE_FORWARD:
        if (dist < WALL_DISTANCE) {
          changeState(STATE_SCAN);
        }
        leftMotor.forward(speed);
        rightMotor.forward(speed);
        moving = true;
        break;

      case STATE_SCAN:
        if (millis() - stateStartTime > SCAN_TIME) {
          changeState(STATE_TURN_LEFT);
        }
        moving = false;
        break;

      case STATE_TURN_LEFT:
        if (millis() - stateStartTime > TURN_TIME) {
          changeState(STATE_FORWARD);
        }
        leftMotor.reverse(speed);
        rightMotor.forward(speed);
        moving = true;
        break;

      case STATE_TURN_RIGHT:
        if (millis() - stateStartTime > TURN_TIME) {
          changeState(STATE_FORWARD);
        }
        leftMotor.forward(speed);
        rightMotor.reverse(speed);
        moving = true;
        break;
    }
  }

  if (moving) {
    leftMotor.runSpeedToPosition();
    rightMotor.runSpeedToPosition();
    if (leftMotor.distanceToGo() <= 0) {
      Serial.print("reset triggd");
      leftMotor.setCurrentPosition(0);
      rightMotor.setCurrentPosition(0);
      leftMotor.moveTo(4076);
      rightMotor.moveTo(-4076);
      leftMotor.setSpeed(speed);
      rightMotor.setSpeed(speed);
    }
  }
}

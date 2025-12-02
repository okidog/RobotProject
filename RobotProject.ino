#include <IRremote.h>
#include <Servo.h>

// ----------------------- Pin Definitions -----------------------
const byte ultrasonicTrigger = A4;
const byte ultrasonicEcho    = A5;

const byte servoPin       = 9;
const byte IR_RECEIVE_PIN = 3;

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
  STATE_TURN
};

RobotState currentState = STATE_FORWARD;
unsigned long stateStartTime = 0;

// Thresholds
const int WALL_DISTANCE = 25; 
const int SCAN_TIME     = 300;
const int TURN_TIME     = 500;

int turnDirection = 1; // +1 = left, -1 = right

// ----------------------- State Helpers -----------------------
void changeState(RobotState newState) {
  currentState = newState;
  stateStartTime = millis();

  switch (newState) {
    case STATE_FORWARD: Serial.println("STATE: FORWARD"); break;
    case STATE_SCAN:    Serial.println("STATE: SCAN");    break;
    case STATE_TURN:    Serial.println("STATE: TURN");    break;
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
  if (leftDist > rightDist) {
    turnDirection = 1;
    Serial.println("Decision: TURN LEFT");
  } else {
    turnDirection = -1;
    Serial.println("Decision: TURN RIGHT");
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
  int dist = ultrasonic.getDistanceCM();

  Serial.print("Front Distance: ");
  Serial.print(dist);
  Serial.println(" cm");

  switch (currentState) {

    case STATE_FORWARD:
      if (dist < WALL_DISTANCE) {
        changeState(STATE_SCAN);
      }
      break;

    case STATE_SCAN:
      performScanAndChooseTurn();
      changeState(STATE_TURN);
      break;

    case STATE_TURN:
      // No motors yet — just simulate with serial
      if (turnDirection > 0)
        Serial.println("Turning LEFT...");
      else
        Serial.println("Turning RIGHT...");

      // simulate turn timing
      delay(TURN_TIME);

      changeState(STATE_FORWARD);
      break;
  }

  delay(50);
}

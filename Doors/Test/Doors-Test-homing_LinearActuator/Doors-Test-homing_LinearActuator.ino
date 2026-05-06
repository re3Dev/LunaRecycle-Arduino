#include <Servo.h>

Servo roboclaw;

const int RC_PIN = 9;

const int MIN_LIMIT_PIN = 3;
const int MAX_LIMIT_PIN = 2;

const int STOP_US    = 1500;
const int EXTEND_US  = 2000;
const int RETRACT_US = 1000;

const unsigned long HOMING_TIMEOUT_MS = 3000;
const unsigned long PAUSE_AT_END_MS = 1000;

bool autoBackAndForth = false;

bool minHit() {
  return digitalRead(MIN_LIMIT_PIN) == LOW;  // pressed = LOW
}

bool maxHit() {
  return digitalRead(MAX_LIMIT_PIN) == LOW;  // pressed = LOW
}

void stopActuator() {
  roboclaw.writeMicroseconds(STOP_US);
}

void extendActuator() {
  roboclaw.writeMicroseconds(EXTEND_US);
}

void retractActuator() {
  roboclaw.writeMicroseconds(RETRACT_US);
}

bool checkSerialStop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    while (Serial.available() > 0) {
      Serial.read();
    }

    if (command == 's' || command == 'S') {
      autoBackAndForth = false;
      stopActuator();
      Serial.println("Auto mode stopped");
      return true;
    }
  }

  return false;
}

bool goMax() {
  Serial.println("Going to MAX...");

  unsigned long startTime = millis();

  while (!maxHit()) {
    if (checkSerialStop()) {
      return false;
    }

    extendActuator();

    if (millis() - startTime > HOMING_TIMEOUT_MS) {
      stopActuator();
      Serial.println("MAX timeout, moving on...");
      return false;
    }

    delay(10);
  }

  stopActuator();
  Serial.println("Reached MAX");
  return true;
}

bool goMin() {
  Serial.println("Going to MIN...");

  unsigned long startTime = millis();

  while (!minHit()) {
    if (checkSerialStop()) {
      return false;
    }

    retractActuator();

    if (millis() - startTime > HOMING_TIMEOUT_MS) {
      stopActuator();
      Serial.println("MIN timeout, moving on...");
      return false;
    }

    delay(10);
  }

  stopActuator();
  Serial.println("Reached MIN");
  return true;
}

void goBackAndForthOnce() {
  goMax();
  delay(PAUSE_AT_END_MS);

  if (!autoBackAndForth) {
    return;
  }

  goMin();
  delay(PAUSE_AT_END_MS);
}

void setup() {
  Serial.begin(9600);

  pinMode(MIN_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MAX_LIMIT_PIN, INPUT_PULLUP);

  roboclaw.attach(RC_PIN);

  stopActuator();
  delay(3000);

  Serial.println("Ready");
  Serial.println("Type 1 + Enter to go MAX once");
  Serial.println("Type 0 + Enter to go MIN once");
  Serial.println("Type 2 + Enter to run back-and-forth forever");
  Serial.println("Type s + Enter to stop auto mode");
}

void loop() {
  if (autoBackAndForth) {
    goBackAndForthOnce();
    return;
  }

  if (Serial.available() > 0) {
    char command = Serial.read();

    while (Serial.available() > 0) {
      Serial.read();
    }

    if (command == '1') {
      goMax();
    }
    else if (command == '0') {
      goMin();
    }
    else if (command == '2') {
      autoBackAndForth = true;
      Serial.println("Auto back-and-forth mode started");
    }
    else if (command == 's' || command == 'S') {
      autoBackAndForth = false;
      stopActuator();
      Serial.println("Stopped");
    }
    else {
      stopActuator();
      Serial.println("Unknown command. Use 1 = MAX, 0 = MIN, 2 = auto, s = stop.");
    }
  }
}

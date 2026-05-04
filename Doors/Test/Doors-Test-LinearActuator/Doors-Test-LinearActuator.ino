#include <Servo.h>

Servo roboclaw;

const int RC_PIN = 9;

const int STOP_US    = 1500;
const int EXTEND_US  = 2000;
const int RETRACT_US = 1000;

const unsigned long FULL_STROKE_MS = 6000;

void setup() {
  roboclaw.attach(RC_PIN);
  roboclaw.writeMicroseconds(STOP_US);
  delay(3000);
}

void loop() {
  // Extend
  roboclaw.writeMicroseconds(EXTEND_US);
  delay(FULL_STROKE_MS);

  // Delay
  roboclaw.writeMicroseconds(STOP_US);
  delay(1000);

  // Retract
  roboclaw.writeMicroseconds(RETRACT_US);
  delay(FULL_STROKE_MS);

  // Delay
  roboclaw.writeMicroseconds(STOP_US);
  delay(1000);
}

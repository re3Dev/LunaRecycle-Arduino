/*
  FPU Upper Assembly Pick-Place Sequence Test

  Commands:
    home   Home the stepper, then move to Bag 1
    pick   Run the full pick-place cycle
    on     Turn vacuum pump on
    off    Turn vacuum pump off
    up     Move servo arm up
    down   Move servo arm down
    status Print current machine status

  Cycle:
    Home -> Bag 1.
    Pick from Bag 1 and drop at shredder 4 times.
    Move to Bag 2, pick once, drop at shredder, then stop.

  Pump safety:
    Never set IN1 LOW and IN2 HIGH. That would reverse and damage the pump.
*/

#include <AccelStepper.h>
#include <Servo.h>

// Pins.
const int stepPin = 3;
const int dirPin = 2;
const int enablePin = 4;
const int ENA1 = 5;
const int IN1 = 6;
const int IN2 = 7;
const int limitPin = 8;
const int servoPin = 9;

// Servo pulse widths, in microseconds.
const int servoMinUs = 500;
const int servoMaxUs = 2500;
const int servoUpUs = 500;
const int servoDownUs = 1800;

// Stepper motion: speed is mm/s, accel is mm/s^2.
const float maxSpeed = 400.0;
const float accel = 500.0;
const float homeSpeed = 50.0;
const int minPulseWidthUs = 2;

// 3GT belt with 18T pulley.
const float beltPitchMm = 3.0;
const int pulleyTeeth = 18;
const int motorStepsPerRev = 200;
const int microsteps = 8;

// Positions, in mm from home. Negative moves away from the switch.
const float homePos = 0.0;
const float bag1Pos = -120.0;
const float shredderPos = -300.0;
const float bag2Pos = -500.0;

const int homeDir = 1;  // Change to -1 if homing moves away from the switch.
const int bag1TripCount = 4;
const int pumpSpeed = 100;

const unsigned long debounceMs = 50;
const unsigned long bagPauseMs = 3000;
const unsigned long shredderPauseMs = 300;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
Servo armServo;

enum State {
  WAIT_HOME,
  HOMING,
  BACK_TO_BAG1,
  READY,
  BAG1_TO_SHREDDER,
  SHREDDER_TO_BAG1,
  SHREDDER_TO_BAG2,
  BAG2_TO_SHREDDER,
  DONE
};

State state = WAIT_HOME;

int lastLimitRead = HIGH;
int limitState = HIGH;
int bag1TripsDone = 0;
unsigned long lastLimitChange = 0;
String serialCmd = "";

void setup() {
  Serial.begin(9600);

  pinMode(limitPin, INPUT_PULLUP);
  pinMode(ENA1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  vacuumOff();

  armServo.attach(servoPin, servoMinUs, servoMaxUs);
  servoUp();

  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(false, false, true);  // Enable pin is active LOW.
  stepper.setMinPulseWidth(minPulseWidthUs);
  stepper.setMaxSpeed(mmToSteps(maxSpeed));
  stepper.setAcceleration(mmToSteps(accel));
  stepper.disableOutputs();

  Serial.println("Ready. Send 'home' first, then 'pick'.");
}

void loop() {
  readLimitSwitch();
  readSerial();
  runState();
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      serialCmd.trim();
      serialCmd.toLowerCase();
      runCommand(serialCmd);
      serialCmd = "";
    } else {
      serialCmd += c;
    }
  }
}

void runCommand(String cmd) {
  if (cmd == "home" || cmd == "h") {
    startHome();
  } else if (cmd == "pick" || cmd == "p") {
    startPick();
  } else if (cmd == "on") {
    vacuumOn();
  } else if (cmd == "off") {
    vacuumOff();
    Serial.println("Motor off.");
  } else if (cmd == "up") {
    servoUp();
  } else if (cmd == "down") {
    servoDown();
  } else if (cmd == "status" || cmd == "s") {
    printStatus();
  } else if (cmd.length() > 0) {
    Serial.println("Use: home, pick, on, off, up, down, status");
  }
}

void runState() {
  if (state == HOMING) {
    runHome();
    return;
  }

  stepper.run();

  if (stepper.distanceToGo() != 0) {
    return;
  }

  switch (state) {
    case BACK_TO_BAG1:
      stepper.disableOutputs();
      state = READY;
      Serial.println("Homed. At Bag 1. Send 'pick'.");
      break;

    case BAG1_TO_SHREDDER:
      pauseAtShredder();
      bag1TripsDone++;
      Serial.print("Bag 1 trip ");
      Serial.print(bag1TripsDone);
      Serial.print(" of ");
      Serial.println(bag1TripCount);

      if (bag1TripsDone < bag1TripCount) {
        moveTo(bag1Pos, SHREDDER_TO_BAG1);
      } else {
        moveTo(bag2Pos, SHREDDER_TO_BAG2);
      }
      break;

    case SHREDDER_TO_BAG1:
      pickAtBag();
      moveTo(shredderPos, BAG1_TO_SHREDDER);
      break;

    case SHREDDER_TO_BAG2:
      pickAtBag();
      moveTo(shredderPos, BAG2_TO_SHREDDER);
      break;

    case BAG2_TO_SHREDDER:
      pauseAtShredder();
      stepper.disableOutputs();
      state = DONE;
      Serial.println("Bag 2 trip done. Cycle complete.");
      break;

    default:
      break;
  }
}

void startHome() {
  bag1TripsDone = 0;
  state = HOMING;

  stepper.enableOutputs();
  stepper.setSpeed(mmToSteps(homeSpeed) * homeDir);

  Serial.println("Homing...");
}

void runHome() {
  if (limitPressed()) {
    stepper.setCurrentPosition(mmToSteps(homePos));
    stepper.setSpeed(0);
    moveTo(bag1Pos, BACK_TO_BAG1);
    Serial.println("Limit switch hit. Moving to Bag 1.");
    return;
  }

  stepper.runSpeed();
}

void startPick() {
  if (state != READY) {
    Serial.println("Home first. Wait until machine is at Bag 1.");
    return;
  }

  bag1TripsDone = 0;
  pickAtBag();
  moveTo(shredderPos, BAG1_TO_SHREDDER);
  Serial.println("Pick cycle started.");
}

void pickAtBag() {
  vacuumOn();
  servoDown();
  delay(bagPauseMs / 2);
  servoUp();
  delay(bagPauseMs - (bagPauseMs / 2));
}

void pauseAtShredder() {
  vacuumOff();
  delay(shredderPauseMs);
}

void moveTo(float targetMm, State nextState) {
  stepper.enableOutputs();
  stepper.moveTo(mmToSteps(targetMm));
  state = nextState;
}

long mmToSteps(float mm) {
  float mmPerRev = beltPitchMm * pulleyTeeth;
  float stepsPerMm = (motorStepsPerRev * microsteps) / mmPerRev;
  return lround(mm * stepsPerMm);
}

float stepsToMm(long steps) {
  float mmPerRev = beltPitchMm * pulleyTeeth;
  float stepsPerMm = (motorStepsPerRev * microsteps) / mmPerRev;
  return steps / stepsPerMm;
}

void readLimitSwitch() {
  int reading = digitalRead(limitPin);

  if (reading != lastLimitRead) {
    lastLimitChange = millis();
  }

  if (millis() - lastLimitChange > debounceMs) {
    limitState = reading;
  }

  lastLimitRead = reading;
}

bool limitPressed() {
  return limitState == LOW;
}

void vacuumOn() {
  // Pump forward only. Do not reverse this pin order.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA1, pumpSpeed);
  Serial.println("Motor on.");
}

void vacuumOff() {
  analogWrite(ENA1, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void servoDown() {
  armServo.writeMicroseconds(servoDownUs);
  Serial.println("Servo down.");
}

void servoUp() {
  armServo.writeMicroseconds(servoUpUs);
  Serial.println("Servo up.");
}

void printStatus() {
  Serial.print("State: ");
  Serial.println(state);

  Serial.print("Position: ");
  Serial.print(stepsToMm(stepper.currentPosition()));
  Serial.println(" mm");

  Serial.print("Limit: ");
  Serial.println(limitPressed() ? "pressed" : "open");

  Serial.print("Bag 1 trips: ");
  Serial.println(bag1TripsDone);
}

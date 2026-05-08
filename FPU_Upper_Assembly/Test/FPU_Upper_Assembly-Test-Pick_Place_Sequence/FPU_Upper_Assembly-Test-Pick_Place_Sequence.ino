/*
  FPU Upper Assembly Pick-Place Sequence Test

  Commands:
    home   Home the stepper, then move to Bag 1
    pick   Run the full pick-place cycle
    on     Turn vacuum pump on
    off    Turn vacuum pump off
    up     Move servo arm up
    down   Move servo arm down
    cycle  Cycle servo between 0 and 180 until another servo position is sent
    y <0 to 180>
           Move servo to a specific angle, in degrees
    x <-550 to -10>
           Move stepper to a specific position, in mm from home
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
const int vacuumSensorPin = A0;

// Servo angles, in degrees, for a 180-degree servo.
const int servoMinDeg = 0;
const int servoMaxDeg = 180;
const int servoUpDeg = 0;
const int servoDownDeg = 180;
const unsigned long servoCycleIntervalMs = 5;

// Vacuum pick settings.
const float analogReferenceV = 5.0;
const float vacuumThresholdV = 2.5;
const bool vacuumDetectedWhenVoltageHigh = true;
const int servoPickStepDeg = 10;
const int servoReturnStepDeg = 5;
const unsigned long pumpPrimeMs = 75;
const unsigned long servoStepSettleMs = 35;
const unsigned long servoReturnStepSettleMs = 35;
const int vacuumConfirmSamples = 3;
const unsigned long vacuumConfirmDelayMs = 15;

// Stepper motion: speed is mm/s, accel is mm/s^2.
const float maxSpeed = 600.0;
const float accel = 700.0;
const float homeSpeed = 60.0;
const int minPulseWidthUs = 2;

// 3GT belt with 18T pulley.
const float beltPitchMm = 3.0;
const int pulleyTeeth = 18;
const int motorStepsPerRev = 200;
const int microsteps = 4;

// Positions, in mm from home. Negative moves away from the switch.
const float homePos = 0.0;
const float bag1Pos = -50.0;
const float shredderPos = -300.0;
const float bag2Pos = -500.0;
const float stepperMinPos = -550.0;
const float stepperMaxPos = -10.0;

const int homeDir = 1;  // Change to -1 if homing moves away from the switch.
const int bag1TripCount = 4;
const int pumpSpeed = 100;

const unsigned long debounceMs = 50;
const unsigned long shredderPauseMs = 1000;

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
  MANUAL_MOVE,
  DONE
};

State state = WAIT_HOME;

int lastLimitRead = HIGH;
int limitState = HIGH;
int bag1TripsDone = 0;
unsigned long lastLimitChange = 0;
String serialCmd = "";
bool servoCycling = false;
int currentServoAngle = servoUpDeg;
int servoCycleDirection = 1;
unsigned long lastServoCycleMove = 0;

void setup() {
  Serial.begin(9600);

  pinMode(limitPin, INPUT_PULLUP);
  pinMode(ENA1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  vacuumOff();

  armServo.attach(servoPin);
  servoUp();

  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(false, false, true);  // Enable pin is active LOW.
  stepper.setMinPulseWidth(minPulseWidthUs);
  stepper.setMaxSpeed(mmToSteps(maxSpeed));
  stepper.setAcceleration(mmToSteps(accel));
  stepper.disableOutputs();

  Serial.println("Ready. Send 'home' first, then 'pick'.");
  Serial.println("Manual moves: y 0 to 180 deg, x -550 to -10 mm.");
}

void loop() {
  readLimitSwitch();
  readSerial();
  updateServoCycle();
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
  } else if (cmd == "cycle") {
    startServoCycle();
  } else if (cmd.startsWith("y ")) {
    handleServoAngleCommand(cmd.substring(2));
  } else if (cmd.startsWith("x ")) {
    handleStepperPositionCommand(cmd.substring(2));
  } else if (isServoAngleCommand(cmd)) {
    moveServoTo(cmd.toInt());
  } else if (isStepperPositionCommand(cmd)) {
    moveStepperTo(cmd.toFloat());
  } else if (isNumberCommand(cmd)) {
    printPositionRangeError(cmd);
  } else if (cmd == "status" || cmd == "s") {
    printStatus();
  } else if (cmd.length() > 0) {
    Serial.println("Use: home, pick, on, off, up, down, cycle, y 0 to 180, x -550 to -10, status");
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

    case MANUAL_MOVE:
      stepper.disableOutputs();
      state = READY;
      Serial.println("Manual stepper move complete.");
      break;

    default:
      break;
  }
}

void startHome() {
  stopServoCycle();
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

  stopServoCycle();
  bag1TripsDone = 0;
  pickAtBag();
  moveTo(shredderPos, BAG1_TO_SHREDDER);
  Serial.println("Pick cycle started.");
}

void pickAtBag() {
  stopServoCycle();
  vacuumOn();
  delay(pumpPrimeMs);

  bool bagGrabbed = false;

  for (int angle = servoUpDeg; angle <= servoDownDeg; angle += servoPickStepDeg) {
    writeServoAngle(angle);
    delay(servoStepSettleMs);

    if (vacuumDetected()) {
      bagGrabbed = true;
      Serial.print("Vacuum detected at servo angle ");
      Serial.print(angle);
      Serial.println(" deg.");
      break;
    }
  }

  if (!bagGrabbed) {
    servoDown();
    delay(servoStepSettleMs);
    Serial.println("No vacuum detected at full down position.");
  }

  returnServoToUp();
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

void moveStepperTo(float targetMm) {
  if (state == WAIT_HOME || state == HOMING) {
    Serial.println("Home first before sending a stepper position.");
    return;
  }

  if (state != READY && state != DONE && state != MANUAL_MOVE) {
    Serial.println("Wait for the current pick-place move to finish before manual stepper moves.");
    return;
  }

  targetMm = constrain(targetMm, stepperMinPos, stepperMaxPos);
  moveTo(targetMm, MANUAL_MOVE);

  Serial.print("Moving stepper to ");
  Serial.print(targetMm);
  Serial.println(" mm.");
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

float rawToVoltage(int raw) {
  return raw * (analogReferenceV / 1023.0);
}

bool vacuumDetected() {
  int detectedSamples = 0;

  for (int i = 0; i < vacuumConfirmSamples; i++) {
    int raw = analogRead(vacuumSensorPin);
    float voltage = rawToVoltage(raw);
    bool detected = vacuumDetectedWhenVoltageHigh
                      ? voltage >= vacuumThresholdV
                      : voltage <= vacuumThresholdV;

    if (detected) {
      detectedSamples++;
    }

    if (i < vacuumConfirmSamples - 1) {
      delay(vacuumConfirmDelayMs);
    }
  }

  return detectedSamples == vacuumConfirmSamples;
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
  stopServoCycle();
  writeServoAngle(servoDownDeg);
  Serial.println("Servo down.");
}

void servoUp() {
  stopServoCycle();
  writeServoAngle(servoUpDeg);
  Serial.println("Servo up.");
}

void returnServoToUp() {
  stopServoCycle();

  while (currentServoAngle > servoUpDeg) {
    int nextAngle = currentServoAngle - servoReturnStepDeg;

    if (nextAngle < servoUpDeg) {
      nextAngle = servoUpDeg;
    }

    writeServoAngle(nextAngle);
    delay(servoReturnStepSettleMs);
  }

  Serial.println("Servo returned up.");
}

void moveServoTo(int angle) {
  stopServoCycle();
  writeServoAngle(angle);

  Serial.print("Moved servo to ");
  Serial.print(currentServoAngle);
  Serial.println(" degrees.");
}

void handleServoAngleCommand(String value) {
  value.trim();

  if (!isServoAngleCommand(value)) {
    Serial.println("Servo angle must be from 0 to 180.");
    return;
  }

  moveServoTo(value.toInt());
}

void handleStepperPositionCommand(String value) {
  value.trim();

  if (!isStepperPositionCommand(value)) {
    Serial.println("Stepper position must be from -550 to -10 mm.");
    return;
  }

  moveStepperTo(value.toFloat());
}

void writeServoAngle(int angle) {
  currentServoAngle = constrain(angle, servoMinDeg, servoMaxDeg);
  armServo.write(currentServoAngle);
}

void startServoCycle() {
  servoCycling = true;
  lastServoCycleMove = millis();
  Serial.println("Cycling servo between 0 and 180 degrees. Send any angle, up, or down to stop.");
}

void stopServoCycle() {
  servoCycling = false;
}

void updateServoCycle() {
  if (!servoCycling || millis() - lastServoCycleMove < servoCycleIntervalMs) {
    return;
  }

  lastServoCycleMove = millis();
  currentServoAngle += servoCycleDirection;

  if (currentServoAngle >= servoMaxDeg) {
    currentServoAngle = servoMaxDeg;
    servoCycleDirection = -1;
  } else if (currentServoAngle <= servoMinDeg) {
    currentServoAngle = servoMinDeg;
    servoCycleDirection = 1;
  }

  armServo.write(currentServoAngle);
}

bool isServoAngleCommand(String cmd) {
  if (cmd.length() == 0) {
    return false;
  }

  for (unsigned int i = 0; i < cmd.length(); i++) {
    if (!isDigit(cmd.charAt(i))) {
      return false;
    }
  }

  int angle = cmd.toInt();
  return angle >= servoMinDeg && angle <= servoMaxDeg;
}

bool isStepperPositionCommand(String cmd) {
  if (!isNumberCommand(cmd)) {
    return false;
  }

  float positionMm = cmd.toFloat();
  return positionMm >= stepperMinPos && positionMm <= stepperMaxPos;
}

bool isNumberCommand(String cmd) {
  if (cmd.length() == 0) {
    return false;
  }

  bool hasDigit = false;
  bool hasDecimal = false;

  for (unsigned int i = 0; i < cmd.length(); i++) {
    char c = cmd.charAt(i);

    if (isDigit(c)) {
      hasDigit = true;
    } else if (c == '-' && i == 0) {
      continue;
    } else if (c == '.' && !hasDecimal) {
      hasDecimal = true;
    } else {
      return false;
    }
  }

  return hasDigit;
}

void printPositionRangeError(String cmd) {
  if (cmd.toFloat() < 0) {
    Serial.println("Stepper position must be from -550 to -10 mm.");
  } else {
    Serial.println("Servo angle must be from 0 to 180.");
  }
}

void printStatus() {
  Serial.print("State: ");
  Serial.println(state);

  Serial.print("Position: ");
  Serial.print(stepsToMm(stepper.currentPosition()));
  Serial.println(" mm");

  Serial.print("Limit: ");
  Serial.println(limitPressed() ? "pressed" : "open");

  int vacuumRaw = analogRead(vacuumSensorPin);
  float vacuumVoltage = rawToVoltage(vacuumRaw);
  Serial.print("Vacuum: ");
  Serial.print(vacuumDetected() ? "detected" : "not detected");
  Serial.print(" | raw: ");
  Serial.print(vacuumRaw);
  Serial.print(" | voltage: ");
  Serial.print(vacuumVoltage, 2);
  Serial.println(" V");

  Serial.print("Bag 1 trips: ");
  Serial.println(bag1TripsDone);

  Serial.print("Servo angle: ");
  Serial.print(currentServoAngle);
  Serial.print(" deg | cycling: ");
  Serial.println(servoCycling ? "yes" : "no");
}

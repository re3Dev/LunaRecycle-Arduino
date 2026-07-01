/*
  FPU Upper Assembly Pick-Place Sequence Test

  Commands:
    home   Home the stepper, then move to Bag 1
    start  Run repeating pick-place sequence: Bag 1 four times, then Bag 2 once
    stop   Stop the current cycle and turn outputs off
    on     Turn vacuum pump on
    off    Turn vacuum pump off
    up     Move servo arm up
    down   Move servo arm down
    ir on  Enable the IR bag stack sensor
    ir off Disable the IR bag stack sensor
    y <0 to 270>
           Move servo to a specific angle, in degrees
    x <-550 to -10>
           Move stepper to a specific position, in mm from home
    status Print current machine status

  Cycle:
    Home -> Bag 1.
    Pick from Bag 1 four times, then Bag 2 once, and repeat.
    Before each pick, the IR sensor confirms the selected bag stack is not empty.
    Missing bag, missing vacuum, or stop command stops the sequence.

  Vacuum pump is controlled by a simple relay.
*/

#include <AccelStepper.h>
#include <Servo.h>

// Pins.
const int TC_stepperMotorController_step = 4;
const int TC_stepperMotorController_dir = 22;
const int TC_stepperMotorController_enable = 23;
const int TC_vacuumPumpRelay = 24;
const int TC_stepperHomeLimit = 34;
const int TC_servoMotor_pin = 5;
const int TC_leftFilmSensor = 35;
const int TC_rightFilmSensor = 36;
const int TC_vacuumSensor = A0;

// Servo angles, in degrees, for a 270-degree servo.
const int servoMinDeg = 0;
const int servoMaxDeg = 270;
const int servoMinUs = 500;
const int servoMaxUs = 2500;
const int servoUpDeg = 0;
const int servoDownDeg = 270;

// Vacuum pick settings.
const float analogReferenceV = 5.0;
const float vacuumThresholdV = 2.5;
const bool vacuumDetectedWhenVoltageHigh = true;
const int servoPickStepDeg = 1;
const unsigned long pumpPrimeMs = 75;
const unsigned long servoStepWaitMs = 6;
const unsigned long vacuumCheckIntervalMs = 1;
const unsigned long servoReturnMsPerDeg = 4;
const unsigned long servoReturnMinMs = 150;
const int vacuumConfirmSamples = 3;
const int vacuumFastConfirmSamples = 2;
const unsigned long vacuumConfirmDelayMs = 1;

// IR bag stack sensor. Most 3-pin IR obstacle sensors read LOW when an object is present.
const bool bagPresentState = LOW;
const int bagSensorConfirmSamples = 5;
const int bagSensorEmptyConfirmCount = 4;
const unsigned long bagSensorConfirmDelayMs = 20;

// Stepper motion: speed is mm/s, accel is mm/s^2.
const float maxSpeed = 600.0;
const float accel = 700.0;
const float homeSpeed = 50.0;
const int minPulseWidthUs = 2;

// 3GT belt with 18T pulley.
const float beltPitchMm = 3.0;
const int pulleyTeeth = 18;
const int motorStepsPerRev = 200;
const int microsteps = 4;

// Positions, in mm from home. Negative moves away from the switch.
const float homePos = 0.0;
const float bag1Pos = -25.0;
const float bag2Pos = -500.0;  // Tune this to the Bag 2 stack position.
const float shredderPos = -475.0;
const float stepperMinPos = -550.0;
const float stepperMaxPos = -10.0;

const int homeDir = 1;  // Change to -1 if homing moves away from the switch.

const unsigned long debounceMs = 50;
const unsigned long shredderPauseMs = 1000;

AccelStepper stepper(AccelStepper::DRIVER, TC_stepperMotorController_step, TC_stepperMotorController_dir);
Servo TC_servoMotor;

enum State {
  WAIT_HOME,
  HOMING,
  BACK_TO_BAG1,
  READY,
  MOVE_TO_BAG,
  BAG_TO_SHREDDER,
  MANUAL_MOVE,
  DONE
};

State state = WAIT_HOME;

int lastLimitRead = HIGH;
int limitState = HIGH;
int bagTripsDone = 0;
int activeBag = 1;
int sequenceStep = 0;
unsigned long lastLimitChange = 0;
String serialCmd = "";
String lastStopReason = "none";
bool irDetectionEnabled = true;
bool sequenceRunning = false;
int currentServoAngle = servoUpDeg;

void setup() {
  Serial.begin(9600);

  pinMode(TC_stepperHomeLimit, INPUT_PULLUP);
  pinMode(TC_leftFilmSensor, INPUT);
  pinMode(TC_rightFilmSensor, INPUT);
  pinMode(TC_vacuumPumpRelay, OUTPUT);

  vacuumOff();

  TC_servoMotor.attach(TC_servoMotor_pin, servoMinUs, servoMaxUs);
  servoUp();

  stepper.setEnablePin(TC_stepperMotorController_enable);
  stepper.setPinsInverted(false, false, true);  // Enable pin is active LOW.
  stepper.setMinPulseWidth(minPulseWidthUs);
  stepper.setMaxSpeed(mmToSteps(maxSpeed));
  stepper.setAcceleration(mmToSteps(accel));
  stepper.disableOutputs();

  Serial.println("Ready. Send 'home' first, then 'start'.");
  Serial.println("Sequence: Bag 1 four times, then Bag 2 once, repeat.");
  Serial.println("Manual moves: y 0 to 270 deg, x -550 to -10 mm.");
  Serial.println("IR detection: ir on, ir off, or ir.");
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
  } else if (cmd == "start" || cmd == "run") {
    startSequence();
  } else if (cmd == "stop") {
    stopPickSequence("Cycle stopped by user.");
  } else if (cmd == "on") {
    vacuumOn();
  } else if (cmd == "off") {
    vacuumOff();
    Serial.println("Motor off.");
  } else if (cmd == "up") {
    servoUp();
  } else if (cmd == "down") {
    servoDown();
  } else if (cmd == "ir on") {
    setIrDetection(true);
  } else if (cmd == "ir off") {
    setIrDetection(false);
  } else if (cmd == "ir") {
    printIrDetectionStatus();
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
    Serial.println("Use: home, start, stop, on, off, up, down, ir on, ir off, y 0 to 270, x -550 to -10, status");
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
      Serial.println("Homed. At Bag 1. Send 'start'.");
      break;

    case MOVE_TO_BAG:
      if (pickAtBag()) {
        moveTo(shredderPos, BAG_TO_SHREDDER);
        Serial.println("Pick complete. Moving to shredder.");
      }
      break;

    case BAG_TO_SHREDDER:
      dropAtShredderAndContinue();
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
  sequenceRunning = false;
  bagTripsDone = 0;
  activeBag = 1;
  sequenceStep = 0;
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

void startSequence() {
  if (state != READY && state != DONE) {
    Serial.println("Home first. Wait until machine is ready.");
    return;
  }

  sequenceRunning = true;
  bagTripsDone = 0;
  sequenceStep = 0;
  lastStopReason = "none";
  activeBag = nextSequenceBag();
  moveTo(activeBagPosition(), MOVE_TO_BAG);
  Serial.print("Sequence started. Moving to Bag ");
  Serial.print(activeBag);
  Serial.println(".");
}

int nextSequenceBag() {
  return sequenceStep < 4 ? 1 : 2;
}

void advanceSequenceStep() {
  sequenceStep++;

  if (sequenceStep >= 5) {
    sequenceStep = 0;
  }
}

void dropAtShredderAndContinue() {
  vacuumOff();
  delay(shredderPauseMs);

  bagTripsDone++;
  Serial.print("Dropped Bag ");
  Serial.print(activeBag);
  Serial.print(" pick at shredder. Total trips: ");
  Serial.println(bagTripsDone);

  advanceSequenceStep();

  if (!sequenceRunning) {
    stepper.disableOutputs();
    state = READY;
    Serial.println("Sequence paused after shredder drop.");
    return;
  }

  activeBag = nextSequenceBag();
  moveTo(activeBagPosition(), MOVE_TO_BAG);

  Serial.print("Next pick: Bag ");
  Serial.print(activeBag);
  Serial.print(" | sequence step ");
  Serial.print(sequenceStep + 1);
  Serial.println(" of 5.");
}

float activeBagPosition() {
  return activeBag == 2 ? bag2Pos : bag1Pos;
}

String activeBagName() {
  return activeBag == 2 ? "Bag 2" : "Bag 1";
}

bool pickAtBag() {
  if (bagStackEmptyConfirmed()) {
    stopPickSequence(activeBagName() + " bag not detected. Sequence stopped.");
    return false;
  }

  vacuumOn();
  delay(pumpPrimeMs);

  bool bagGrabbed = false;

  for (int angle = servoUpDeg; angle <= servoDownDeg; angle += servoPickStepDeg) {
    writeServoAngle(angle);

    if (vacuumDetectedDuringWait()) {
      bagGrabbed = true;
      Serial.print("Vacuum detected at servo angle ");
      Serial.print(currentServoAngle);
      Serial.println(" deg.");
      break;
    }
  }

  if (!bagGrabbed) {
    servoDown();
    delay(servoStepWaitMs);
    stopPickSequence("Vacuum not detected. Sequence stopped.");
    return false;
  }

  returnServoToUp();
  return true;
}

void stopPickSequence(String message) {
  sequenceRunning = false;
  vacuumOff();
  stepper.stop();
  stepper.disableOutputs();
  state = DONE;
  lastStopReason = message;
  printStopReason(message);
}

void printStopReason(String message) {
  Serial.print("Stopped: ");
  Serial.println(message);

  Serial.print("Active bag: ");
  Serial.println(activeBag);

  Serial.print("Sequence step: ");
  Serial.print(sequenceStep + 1);
  Serial.println(" of 5");

  Serial.print("Position: ");
  Serial.print(stepsToMm(stepper.currentPosition()));
  Serial.println(" mm");

  Serial.print("Limit: ");
  Serial.println(limitPressed() ? "pressed" : "open");

  Serial.print("Bag sensor: ");
  if (irDetectionEnabled) {
    Serial.println(digitalRead(activeBagSensorPin()) == bagPresentState ? "bag detected" : "bag not detected");
  } else {
    Serial.println("IR detection disabled");
  }

  int vacuumRaw = analogRead(TC_vacuumSensor);
  float vacuumVoltage = rawToVoltage(vacuumRaw);
  Serial.print("Vacuum: ");
  Serial.print(vacuumDetected() ? "detected" : "not detected");
  Serial.print(" | raw: ");
  Serial.print(vacuumRaw);
  Serial.print(" | voltage: ");
  Serial.print(vacuumVoltage, 2);
  Serial.println(" V");
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
  sequenceRunning = false;
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
  int reading = digitalRead(TC_stepperHomeLimit);

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
    if (vacuumDetectedOnce()) {
      detectedSamples++;
    }

    if (i < vacuumConfirmSamples - 1) {
      delay(vacuumConfirmDelayMs);
    }
  }

  return detectedSamples == vacuumConfirmSamples;
}

bool vacuumDetectedDuringWait() {
  unsigned long waitStart = millis();
  int consecutiveDetections = 0;

  while (millis() - waitStart < servoStepWaitMs) {
    if (vacuumDetectedOnce()) {
      consecutiveDetections++;
    } else {
      consecutiveDetections = 0;
    }

    if (consecutiveDetections >= vacuumFastConfirmSamples) {
      return true;
    }

    delay(vacuumCheckIntervalMs);
  }

  return false;
}

bool vacuumDetectedOnce() {
  int raw = analogRead(TC_vacuumSensor);
  float voltage = rawToVoltage(raw);

  return vacuumDetectedWhenVoltageHigh
           ? voltage >= vacuumThresholdV
           : voltage <= vacuumThresholdV;
}

bool bagStackEmptyConfirmed() {
  if (!irDetectionEnabled) {
    return false;
  }

  int emptySamples = 0;

  for (int i = 0; i < bagSensorConfirmSamples; i++) {
    bool bagPresent = digitalRead(activeBagSensorPin()) == bagPresentState;

    if (!bagPresent) {
      emptySamples++;
    }

    if (i < bagSensorConfirmSamples - 1) {
      delay(bagSensorConfirmDelayMs);
    }
  }

  return emptySamples >= bagSensorEmptyConfirmCount;
}

void setIrDetection(bool enabled) {
  irDetectionEnabled = enabled;
  printIrDetectionStatus();
}

void printIrDetectionStatus() {
  Serial.print("IR detection: ");
  Serial.println(irDetectionEnabled ? "enabled" : "disabled");
}

void vacuumOn() {
  digitalWrite(TC_vacuumPumpRelay, HIGH);
  Serial.println("Motor on.");
}

int activeBagSensorPin() {
  return activeBag == 2 ? TC_rightFilmSensor : TC_leftFilmSensor;
}

void vacuumOff() {
  digitalWrite(TC_vacuumPumpRelay, LOW);
}

void servoDown() {
  writeServoAngle(servoDownDeg);
  Serial.println("Servo down.");
}

void servoUp() {
  writeServoAngle(servoUpDeg);
  Serial.println("Servo up.");
}

void returnServoToUp() {
  int returnDistance = abs(currentServoAngle - servoUpDeg);
  writeServoAngle(servoUpDeg);
  delay(max(servoReturnMinMs, returnDistance * servoReturnMsPerDeg));

  Serial.println("Servo returned up.");
}

void moveServoTo(int angle) {
  writeServoAngle(angle);

  Serial.print("Moved servo to ");
  Serial.print(currentServoAngle);
  Serial.println(" degrees.");
}

void handleServoAngleCommand(String value) {
  value.trim();

  if (!isServoAngleCommand(value)) {
    Serial.println("Servo angle must be from 0 to 270.");
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
  int pulseWidth = map(currentServoAngle, servoMinDeg, servoMaxDeg, servoMinUs, servoMaxUs);
  TC_servoMotor.writeMicroseconds(pulseWidth);
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
    Serial.println("Servo angle must be from 0 to 270.");
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

  Serial.print("Bag stack: ");
  if (irDetectionEnabled) {
    Serial.println(bagStackEmptyConfirmed() ? "empty" : "bag detected");
  } else {
    Serial.println("IR detection disabled");
  }

  int vacuumRaw = analogRead(TC_vacuumSensor);
  float vacuumVoltage = rawToVoltage(vacuumRaw);
  Serial.print("Vacuum: ");
  Serial.print(vacuumDetected() ? "detected" : "not detected");
  Serial.print(" | raw: ");
  Serial.print(vacuumRaw);
  Serial.print(" | voltage: ");
  Serial.print(vacuumVoltage, 2);
  Serial.println(" V");

  Serial.print("Active bag: ");
  Serial.println(activeBag);

  Serial.print("Sequence running: ");
  Serial.println(sequenceRunning ? "yes" : "no");

  Serial.print("Last stop reason: ");
  Serial.println(lastStopReason);

  Serial.print("Sequence step: ");
  Serial.print(sequenceStep + 1);
  Serial.println(" of 5");

  Serial.print("Bag trips: ");
  Serial.println(bagTripsDone);

  Serial.print("Servo angle: ");
  Serial.print(currentServoAngle);
  Serial.println(" deg");
}

/*
 * LunaRecycle – Combined Production Firmware
 * Copyright (C) re:3D, Inc. — All Rights Reserved
 *
 * Subsystems combined:
 *   - Doors      : 2 linear actuators (RoboClaw RC PWM) + limit switches per actuator
 *   - ShredderGate: 2 RC servos (open/close positions)
 *   - FPU        : Stepper + vacuum pump + arm servo (pick-place sequence)
 *
 * Serial protocol (9600 baud, newline terminated):
 *   All commands are plain-text strings followed by '\n'.
 *   Responses are plain-text lines ending with '\n'.
 *
 *   ── Doors ─────────────────────────────────────────────────────────────────
 *   DOOR_OPEN            Extend both actuators to MAX (limit switch)
 *   DOOR_CLOSE           Retract both actuators to MIN (limit switch)
 *   DOOR_STOP            Stop both actuators immediately
 *   DOOR_STATUS          Print door limit-switch state
 *
 *   ── Shredder Gate ─────────────────────────────────────────────────────────
 *   GATE_OPEN            Move both servos to open position (55°)
 *   GATE_CLOSE           Move both servos to closed position (0°)
 *   GATE_STATUS          Print current servo state
 *
 *   ── FPU (pick-place) ──────────────────────────────────────────────────────
 *   FPU_HOME             Home stepper, move to Bag 1
 *   FPU_PICK             Run full pick-place cycle
 *   FPU_VACUUM_ON        Vacuum pump on
 *   FPU_VACUUM_OFF       Vacuum pump off
 *   FPU_ARM_UP           Servo arm up
 *   FPU_ARM_DOWN         Servo arm down
 *   FPU_STATUS           Print current FPU state
 *   FPU_STOP             Abort current motion (disables stepper)
 *
 *   ── System ────────────────────────────────────────────────────────────────
 *   STATUS               Print all subsystem states as JSON-like string
 *   ESTOP                Emergency stop all motion
 *
 * All text responses are prefixed with the subsystem tag for easy parsing,
 * e.g.:  [DOOR] Reached MAX  or  [FPU] Homing...
 */

// ════════════════════════════════════════════════════════════════════════════
//  Libraries
// ════════════════════════════════════════════════════════════════════════════
#include <Servo.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// ════════════════════════════════════════════════════════════════════════════
//  Pin assignments
// ════════════════════════════════════════════════════════════════════════════

// Doors — linear actuators (RoboClaw RC PWM)
// Actuator A (left door)
const int DOOR_A_RC_PIN      = 44;
const int DOOR_A_MIN_PIN     = 22;  // limit switch MIN (retracted)
const int DOOR_A_MAX_PIN     = 23;  // limit switch MAX (extended)

// Actuator B (right door)
const int DOOR_B_RC_PIN      = 45;
const int DOOR_B_MIN_PIN     = 24;
const int DOOR_B_MAX_PIN     = 25;

// Shredder gate servos
const int GATE_SERVO1_PIN    = 46;
const int GATE_SERVO2_PIN    = 47;

// FPU — stepper driver
const int FPU_STEP_PIN       = 3;
const int FPU_DIR_PIN        = 2;
const int FPU_ENABLE_PIN     = 4;
const int FPU_LIMIT_PIN      = 8;

// FPU — vacuum pump (H-bridge)
const int FPU_ENA1_PIN       = 5;
const int FPU_IN1_PIN        = 6;
const int FPU_IN2_PIN        = 7;

// FPU — arm servo
const int FPU_SERVO_PIN      = 9;

// ════════════════════════════════════════════════════════════════════════════
//  Constants
// ════════════════════════════════════════════════════════════════════════════

// Door actuator RC pulse widths (µs)
const int DOOR_STOP_US    = 1500;
const int DOOR_EXTEND_US  = 2000;
const int DOOR_RETRACT_US = 1000;

const unsigned long DOOR_HOMING_TIMEOUT_MS = 6000;

// Shredder gate servo positions
const int GATE_OPEN_DEG  = 55;
const int GATE_CLOSE_DEG = 0;

// FPU servo pulse widths (µs)
const int FPU_SERVO_MIN_US  = 500;
const int FPU_SERVO_MAX_US  = 2500;
const int FPU_SERVO_UP_US   = 500;
const int FPU_SERVO_DOWN_US = 1800;

// FPU stepper motion
const float FPU_MAX_SPEED   = 400.0;   // mm/s
const float FPU_ACCEL       = 500.0;   // mm/s²
const float FPU_HOME_SPEED  = 50.0;    // mm/s
const int   FPU_MIN_PULSE   = 2;       // µs

// 3GT belt, 18T pulley, 200-step motor, 1/8 microstepping
const float FPU_BELT_PITCH  = 3.0;
const int   FPU_PULLEY_T    = 18;
const int   FPU_STEPS_REV   = 200;
const int   FPU_MICROSTEPS  = 8;

// FPU positions (mm from home; negative = away from switch)
const float FPU_HOME_POS      = 0.0;
const float FPU_BAG1_POS      = -120.0;
const float FPU_SHREDDER_POS  = -300.0;
const float FPU_BAG2_POS      = -500.0;
const int   FPU_HOME_DIR      = 1;
const int   FPU_BAG1_TRIPS    = 4;
const int   FPU_PUMP_SPEED    = 100;

const unsigned long FPU_DEBOUNCE_MS       = 50;
const unsigned long FPU_BAG_PAUSE_MS      = 3000;
const unsigned long FPU_SHREDDER_PAUSE_MS = 300;

// INA219 dryer motor current sensor (optional, reports via STATUS)
// Uncomment the line below if the sensor is wired on the dryer drum motor
// #define HAS_INA219

// ════════════════════════════════════════════════════════════════════════════
//  Objects
// ════════════════════════════════════════════════════════════════════════════

Servo doorServoA;
Servo doorServoB;

Servo gateServo1;
Servo gateServo2;

AccelStepper fpuStepper(AccelStepper::DRIVER, FPU_STEP_PIN, FPU_DIR_PIN);
Servo fpuArmServo;

#ifdef HAS_INA219
Adafruit_INA219 ina219;
#endif

// ════════════════════════════════════════════════════════════════════════════
//  State
// ════════════════════════════════════════════════════════════════════════════

// Doors
enum DoorState { DOOR_IDLE, DOOR_OPENING, DOOR_CLOSING };
DoorState doorState = DOOR_IDLE;

bool gateOpen = false;

// FPU
enum FpuState {
  FPU_WAIT_HOME,
  FPU_HOMING,
  FPU_BACK_TO_BAG1,
  FPU_READY,
  FPU_BAG1_TO_SHREDDER,
  FPU_SHREDDER_TO_BAG1,
  FPU_SHREDDER_TO_BAG2,
  FPU_BAG2_TO_SHREDDER,
  FPU_DONE
};
FpuState fpuState = FPU_WAIT_HOME;
int fpuBag1TripsDone = 0;

int fpuLimitLastRead  = HIGH;
int fpuLimitState     = HIGH;
unsigned long fpuLimitLastChange = 0;

// Serial command buffer
String cmdBuffer = "";

// ════════════════════════════════════════════════════════════════════════════
//  Helpers: unit conversion
// ════════════════════════════════════════════════════════════════════════════

long mmToSteps(float mm) {
  float mmPerRev = FPU_BELT_PITCH * FPU_PULLEY_T;
  float stepsPerMm = ((float)(FPU_STEPS_REV * FPU_MICROSTEPS)) / mmPerRev;
  return lround(mm * stepsPerMm);
}

// ════════════════════════════════════════════════════════════════════════════
//  Door helpers
// ════════════════════════════════════════════════════════════════════════════

bool doorAMinHit() { return digitalRead(DOOR_A_MIN_PIN) == LOW; }
bool doorAMaxHit() { return digitalRead(DOOR_A_MAX_PIN) == LOW; }
bool doorBMinHit() { return digitalRead(DOOR_B_MIN_PIN) == LOW; }
bool doorBMaxHit() { return digitalRead(DOOR_B_MAX_PIN) == LOW; }

void doorAStop()    { doorServoA.writeMicroseconds(DOOR_STOP_US); }
void doorAExtend()  { doorServoA.writeMicroseconds(DOOR_EXTEND_US); }
void doorARetract() { doorServoA.writeMicroseconds(DOOR_RETRACT_US); }
void doorBStop()    { doorServoB.writeMicroseconds(DOOR_STOP_US); }
void doorBExtend()  { doorServoB.writeMicroseconds(DOOR_EXTEND_US); }
void doorBRetract() { doorServoB.writeMicroseconds(DOOR_RETRACT_US); }

void doorsStop() {
  doorAStop();
  doorBStop();
  doorState = DOOR_IDLE;
  Serial.println(F("[DOOR] Stopped"));
}

void doorsStartOpen() {
  if (!doorAMaxHit()) doorAExtend();
  if (!doorBMaxHit()) doorBExtend();
  doorState = DOOR_OPENING;
  Serial.println(F("[DOOR] Opening..."));
}

void doorsStartClose() {
  if (!doorAMinHit()) doorARetract();
  if (!doorBMinHit()) doorBRetract();
  doorState = DOOR_CLOSING;
  Serial.println(F("[DOOR] Closing..."));
}

void doorStatus() {
  Serial.print(F("[DOOR] A_MIN="));
  Serial.print(doorAMinHit() ? 1 : 0);
  Serial.print(F(" A_MAX="));
  Serial.print(doorAMaxHit() ? 1 : 0);
  Serial.print(F(" B_MIN="));
  Serial.print(doorBMinHit() ? 1 : 0);
  Serial.print(F(" B_MAX="));
  Serial.println(doorBMaxHit() ? 1 : 0);
}

// Advance door state machine (non-blocking)
void doorTick() {
  if (doorState == DOOR_OPENING) {
    bool aDone = doorAMaxHit();
    bool bDone = doorBMaxHit();
    if (aDone) doorAStop();
    else doorAExtend();
    if (bDone) doorBStop();
    else doorBExtend();
    if (aDone && bDone) {
      doorState = DOOR_IDLE;
      Serial.println(F("[DOOR] OPEN — both MAX reached"));
    }
  } else if (doorState == DOOR_CLOSING) {
    bool aDone = doorAMinHit();
    bool bDone = doorBMinHit();
    if (aDone) doorAStop();
    else doorARetract();
    if (bDone) doorBStop();
    else doorBRetract();
    if (aDone && bDone) {
      doorState = DOOR_IDLE;
      Serial.println(F("[DOOR] CLOSED — both MIN reached"));
    }
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  Shredder gate helpers
// ════════════════════════════════════════════════════════════════════════════

void gateOpenCmd() {
  gateServo1.write(GATE_OPEN_DEG);
  gateServo2.write(GATE_OPEN_DEG);
  gateOpen = true;
  Serial.println(F("[GATE] OPEN"));
}

void gateCloseCmd() {
  gateServo1.write(GATE_CLOSE_DEG);
  gateServo2.write(GATE_CLOSE_DEG);
  gateOpen = false;
  Serial.println(F("[GATE] CLOSED"));
}

void gateStatus() {
  Serial.print(F("[GATE] state="));
  Serial.println(gateOpen ? F("OPEN") : F("CLOSED"));
}

// ════════════════════════════════════════════════════════════════════════════
//  FPU helpers
// ════════════════════════════════════════════════════════════════════════════

void fpuVacuumOn() {
  digitalWrite(FPU_IN1_PIN, HIGH);
  digitalWrite(FPU_IN2_PIN, LOW);
  analogWrite(FPU_ENA1_PIN, FPU_PUMP_SPEED);
  Serial.println(F("[FPU] Vacuum ON"));
}

void fpuVacuumOff() {
  digitalWrite(FPU_IN1_PIN, LOW);
  digitalWrite(FPU_IN2_PIN, LOW);
  analogWrite(FPU_ENA1_PIN, 0);
  Serial.println(F("[FPU] Vacuum OFF"));
}

void fpuArmUp() {
  fpuArmServo.writeMicroseconds(FPU_SERVO_UP_US);
  Serial.println(F("[FPU] Arm UP"));
}

void fpuArmDown() {
  fpuArmServo.writeMicroseconds(FPU_SERVO_DOWN_US);
  Serial.println(F("[FPU] Arm DOWN"));
}

void fpuMoveTo(float targetMm, FpuState nextState) {
  fpuStepper.enableOutputs();
  fpuStepper.moveTo(mmToSteps(targetMm));
  fpuState = nextState;
}

void fpuPickAtBag() {
  fpuVacuumOn();
  fpuArmServo.writeMicroseconds(FPU_SERVO_DOWN_US);
  delay(FPU_BAG_PAUSE_MS / 2);
  fpuArmServo.writeMicroseconds(FPU_SERVO_UP_US);
  delay(FPU_BAG_PAUSE_MS - (FPU_BAG_PAUSE_MS / 2));
}

void fpuDropAtShredder() {
  fpuVacuumOff();
  delay(FPU_SHREDDER_PAUSE_MS);
}

void fpuReadLimitSwitch() {
  int reading = digitalRead(FPU_LIMIT_PIN);
  if (reading != fpuLimitLastRead) fpuLimitLastChange = millis();
  if (millis() - fpuLimitLastChange > FPU_DEBOUNCE_MS) fpuLimitState = reading;
  fpuLimitLastRead = reading;
}

bool fpuLimitPressed() { return fpuLimitState == LOW; }

void fpuStartHome() {
  fpuBag1TripsDone = 0;
  fpuState = FPU_HOMING;
  fpuStepper.enableOutputs();
  fpuStepper.setSpeed(mmToSteps(FPU_HOME_SPEED) * FPU_HOME_DIR);
  Serial.println(F("[FPU] Homing..."));
}

void fpuStartPick() {
  if (fpuState != FPU_READY) {
    Serial.println(F("[FPU] ERROR: Home first."));
    return;
  }
  fpuBag1TripsDone = 0;
  fpuPickAtBag();
  fpuMoveTo(FPU_SHREDDER_POS, FPU_BAG1_TO_SHREDDER);
  Serial.println(F("[FPU] Pick cycle started"));
}

void fpuStop() {
  fpuStepper.stop();
  fpuStepper.disableOutputs();
  fpuVacuumOff();
  fpuArmServo.writeMicroseconds(FPU_SERVO_UP_US);
  fpuState = FPU_WAIT_HOME;
  Serial.println(F("[FPU] STOPPED"));
}

void fpuPrintStatus() {
  const char* stateStr;
  switch (fpuState) {
    case FPU_WAIT_HOME:          stateStr = "WAIT_HOME"; break;
    case FPU_HOMING:             stateStr = "HOMING"; break;
    case FPU_BACK_TO_BAG1:       stateStr = "BACK_TO_BAG1"; break;
    case FPU_READY:              stateStr = "READY"; break;
    case FPU_BAG1_TO_SHREDDER:   stateStr = "BAG1_TO_SHREDDER"; break;
    case FPU_SHREDDER_TO_BAG1:   stateStr = "SHREDDER_TO_BAG1"; break;
    case FPU_SHREDDER_TO_BAG2:   stateStr = "SHREDDER_TO_BAG2"; break;
    case FPU_BAG2_TO_SHREDDER:   stateStr = "BAG2_TO_SHREDDER"; break;
    case FPU_DONE:               stateStr = "DONE"; break;
    default:                     stateStr = "UNKNOWN"; break;
  }
  Serial.print(F("[FPU] state="));
  Serial.print(stateStr);
  Serial.print(F(" bag1_trips="));
  Serial.println(fpuBag1TripsDone);
}

// Advance FPU state machine (non-blocking)
void fpuTick() {
  fpuReadLimitSwitch();

  if (fpuState == FPU_HOMING) {
    if (fpuLimitPressed()) {
      fpuStepper.setCurrentPosition(mmToSteps(FPU_HOME_POS));
      fpuStepper.setSpeed(0);
      fpuMoveTo(FPU_BAG1_POS, FPU_BACK_TO_BAG1);
      Serial.println(F("[FPU] Limit hit. Moving to Bag 1"));
    } else {
      fpuStepper.runSpeed();
    }
    return;
  }

  fpuStepper.run();

  if (fpuStepper.distanceToGo() != 0) return;

  switch (fpuState) {
    case FPU_BACK_TO_BAG1:
      fpuStepper.disableOutputs();
      fpuState = FPU_READY;
      Serial.println(F("[FPU] At Bag 1. Ready. Send FPU_PICK."));
      break;

    case FPU_BAG1_TO_SHREDDER:
      fpuDropAtShredder();
      fpuBag1TripsDone++;
      Serial.print(F("[FPU] Bag1 trip "));
      Serial.print(fpuBag1TripsDone);
      Serial.print(F("/"));
      Serial.println(FPU_BAG1_TRIPS);
      if (fpuBag1TripsDone < FPU_BAG1_TRIPS) {
        fpuMoveTo(FPU_BAG1_POS, FPU_SHREDDER_TO_BAG1);
      } else {
        fpuMoveTo(FPU_BAG2_POS, FPU_SHREDDER_TO_BAG2);
      }
      break;

    case FPU_SHREDDER_TO_BAG1:
      fpuPickAtBag();
      fpuMoveTo(FPU_SHREDDER_POS, FPU_BAG1_TO_SHREDDER);
      break;

    case FPU_SHREDDER_TO_BAG2:
      fpuPickAtBag();
      fpuMoveTo(FPU_SHREDDER_POS, FPU_BAG2_TO_SHREDDER);
      break;

    case FPU_BAG2_TO_SHREDDER:
      fpuDropAtShredder();
      fpuStepper.disableOutputs();
      fpuState = FPU_DONE;
      Serial.println(F("[FPU] Cycle COMPLETE"));
      break;

    default:
      break;
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  Global STATUS
// ════════════════════════════════════════════════════════════════════════════

void printAllStatus() {
  // Simple space-delimited key=value line — easy to parse in Python
  Serial.print(F("[STATUS]"));
  Serial.print(F(" door_A_min=")); Serial.print(doorAMinHit() ? 1 : 0);
  Serial.print(F(" door_A_max=")); Serial.print(doorAMaxHit() ? 1 : 0);
  Serial.print(F(" door_B_min=")); Serial.print(doorBMinHit() ? 1 : 0);
  Serial.print(F(" door_B_max=")); Serial.print(doorBMaxHit() ? 1 : 0);
  const char* ds;
  if (doorState == DOOR_OPENING) ds = "OPENING";
  else if (doorState == DOOR_CLOSING) ds = "CLOSING";
  else ds = "IDLE";
  Serial.print(F(" door_state=")); Serial.print(ds);

  Serial.print(F(" gate=")); Serial.print(gateOpen ? F("OPEN") : F("CLOSED"));

  const char* fs;
  switch (fpuState) {
    case FPU_WAIT_HOME:         fs = "WAIT_HOME"; break;
    case FPU_HOMING:            fs = "HOMING"; break;
    case FPU_BACK_TO_BAG1:      fs = "BACK_TO_BAG1"; break;
    case FPU_READY:             fs = "READY"; break;
    case FPU_BAG1_TO_SHREDDER:  fs = "BAG1_TO_SHREDDER"; break;
    case FPU_SHREDDER_TO_BAG1:  fs = "SHREDDER_TO_BAG1"; break;
    case FPU_SHREDDER_TO_BAG2:  fs = "SHREDDER_TO_BAG2"; break;
    case FPU_BAG2_TO_SHREDDER:  fs = "BAG2_TO_SHREDDER"; break;
    case FPU_DONE:              fs = "DONE"; break;
    default:                    fs = "UNKNOWN"; break;
  }
  Serial.print(F(" fpu_state=")); Serial.print(fs);
  Serial.print(F(" fpu_trips=")); Serial.println(fpuBag1TripsDone);
}

// ════════════════════════════════════════════════════════════════════════════
//  Command dispatch
// ════════════════════════════════════════════════════════════════════════════

void handleCommand(const String& cmd) {
  if (cmd == "DOOR_OPEN")       { doorsStartOpen(); }
  else if (cmd == "DOOR_CLOSE") { doorsStartClose(); }
  else if (cmd == "DOOR_STOP")  { doorsStop(); }
  else if (cmd == "DOOR_STATUS"){ doorStatus(); }

  else if (cmd == "GATE_OPEN")  { gateOpenCmd(); }
  else if (cmd == "GATE_CLOSE") { gateCloseCmd(); }
  else if (cmd == "GATE_STATUS"){ gateStatus(); }

  else if (cmd == "FPU_HOME")      { fpuStartHome(); }
  else if (cmd == "FPU_PICK")      { fpuStartPick(); }
  else if (cmd == "FPU_VACUUM_ON") { fpuVacuumOn(); }
  else if (cmd == "FPU_VACUUM_OFF"){ fpuVacuumOff(); }
  else if (cmd == "FPU_ARM_UP")    { fpuArmUp(); }
  else if (cmd == "FPU_ARM_DOWN")  { fpuArmDown(); }
  else if (cmd == "FPU_STATUS")    { fpuPrintStatus(); }
  else if (cmd == "FPU_STOP")      { fpuStop(); }

  else if (cmd == "STATUS")  { printAllStatus(); }
  else if (cmd == "ESTOP") {
    doorsStop();
    fpuStop();
    gateCloseCmd();
    Serial.println(F("[SYSTEM] ESTOP — all motion stopped"));
  }
  else if (cmd.length() > 0) {
    Serial.print(F("[SYSTEM] Unknown command: "));
    Serial.println(cmd);
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuffer.trim();
      cmdBuffer.toUpperCase();
      if (cmdBuffer.length() > 0) {
        handleCommand(cmdBuffer);
      }
      cmdBuffer = "";
    } else {
      cmdBuffer += c;
    }
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  Setup / Loop
// ════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(9600);

  // Doors — limit switch pins
  pinMode(DOOR_A_MIN_PIN, INPUT_PULLUP);
  pinMode(DOOR_A_MAX_PIN, INPUT_PULLUP);
  pinMode(DOOR_B_MIN_PIN, INPUT_PULLUP);
  pinMode(DOOR_B_MAX_PIN, INPUT_PULLUP);

  // Doors — RC servos
  doorServoA.attach(DOOR_A_RC_PIN);
  doorServoB.attach(DOOR_B_RC_PIN);
  doorAStop();
  doorBStop();

  // Shredder gate servos
  gateServo1.attach(GATE_SERVO1_PIN);
  gateServo2.attach(GATE_SERVO2_PIN);
  gateServo1.write(GATE_CLOSE_DEG);
  gateServo2.write(GATE_CLOSE_DEG);

  // FPU — pump
  pinMode(FPU_ENA1_PIN, OUTPUT);
  pinMode(FPU_IN1_PIN, OUTPUT);
  pinMode(FPU_IN2_PIN, OUTPUT);
  fpuVacuumOff();

  // FPU — limit switch
  pinMode(FPU_LIMIT_PIN, INPUT_PULLUP);

  // FPU — arm servo
  fpuArmServo.attach(FPU_SERVO_PIN, FPU_SERVO_MIN_US, FPU_SERVO_MAX_US);
  fpuArmServo.writeMicroseconds(FPU_SERVO_UP_US);

  // FPU — stepper
  fpuStepper.setEnablePin(FPU_ENABLE_PIN);
  fpuStepper.setPinsInverted(false, false, true);  // enable pin active LOW
  fpuStepper.setMinPulseWidth(FPU_MIN_PULSE);
  fpuStepper.setMaxSpeed(mmToSteps(FPU_MAX_SPEED));
  fpuStepper.setAcceleration(mmToSteps(FPU_ACCEL));
  fpuStepper.disableOutputs();

#ifdef HAS_INA219
  Wire.begin();
  if (!ina219.begin()) {
    Serial.println(F("[SYSTEM] WARNING: INA219 not found"));
  }
#endif

  Serial.println(F("[SYSTEM] LunaRecycle firmware ready"));
  Serial.println(F("[SYSTEM] Send STATUS for current state"));
}

void loop() {
  readSerial();
  doorTick();
  fpuTick();
}

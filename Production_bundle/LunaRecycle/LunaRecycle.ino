/*
 * LunaRecycle - Arduino Mega 2560 Production Firmware
 * Copyright (C) re:3D, Inc. - All Rights Reserved
 *
 * Target board : Arduino Mega 2560
 *
 * Subsystems:
 *   - Shredder Gate   : 2 RC servos (open / close)
 *   - Screw Motor     : H-bridge speed + direction (continuous spin)
 *   - Energy Monitor  : INA219 current / voltage / power sensor
 *   - Trash Conveyor  : stepper pick-and-place (bag stack -> shredder)
 *
 * -- Serial protocol (9600 baud, newline-terminated) -------------------------
 *
 *   GATE_OPEN              Move both servos to open position (55 deg)
 *   GATE_CLOSE             Move both servos to closed position (0 deg)
 *   GATE_STATUS            Print current gate state
 *
 *   MOTOR_SET <spd> <dir>  Set screw motor.  spd = 0-255,  dir = FWD or REV
 *   MOTOR_STOP             Stop screw motor (PWM = 0)
 *   MOTOR_STATUS           Print current motor speed / direction
 *
 *   TC_HOME                Home the conveyor stepper, then park at Bag 1
 *   TC_PICK                Run the repeating sequence: Bag 1 x4, then Bag 2 x1
 *   TC_STOP                Stop the pick cycle
 *   TC_UP / TC_DOWN        Raise / lower the picker servo
 *   TC_SERVO <0-270>       Move picker servo to an angle (deg)
 *   TC_PUMP_ON / _OFF      Manually switch the vacuum pump relay
 *   TC_IR_ON / _OFF        Enable / disable the IR bag-stack sensor check
 *   TC_MOVE <-550..-10>    Manual stepper move (mm from home)
 *   TC_STATUS              Print trash-conveyor detail status
 *
 *   SHREDDER_ON / _OFF     Switch the shredder motor on / off
 *   SHREDDER_FWD / _REV    Set shredder motor direction
 *
 *   BLASTGATE_HOME <L|R|ALL>     Retract gate to MIN (0%)
 *   BLASTGATE_HOMEMAX <L|R|ALL>  Extend gate to MAX (100%)
 *   BLASTGATE_CAL <L|R|ALL>      Home then time a full stroke (calibrate)
 *   BLASTGATE_POS <L|R> <0-100>  Move gate to a % of its stroke
 *   BLASTGATE_EXT/RET <L|R> <ms> Jog extend / retract for <ms>
 *   BLASTGATE_SPEED <1-100>      Set blast gate motion speed (% of full)
 *   BLASTGATE_STOP               Stop both blast gate actuators
 *   BLASTGATE_STATUS             Print blast gate calibration + position
 *
 *   STATUS                 Print all subsystem states + one INA219 snapshot
 *   ESTOP                  Stop motor, close gate, halt conveyor immediately
 *
 * INA219 readings are also streamed automatically every 500 ms:
 *   [ENERGY] pwm=150 dir=FWD V=12.34 I=1.234 P=15.23
 *
 * -- Pin map (Mega 2560) ------------------------------------------------------
 *
 *   Gate servo L / R   ->  D9  / D10
 *   Screw motor ENA    ->  D3  (PWM)
 *   Screw motor IN1/2  ->  D7  / D8
 *   INA219             ->  D20 (SDA) / D21 (SCL)   [hardware I2C]
 *
 *   Mixer blast gates (RoboClaw RC pulse):
 *     LEFT  actuator  ->  D9,  limits MIN=D37 MAX=D38
 *     RIGHT actuator  ->  D8,  limits MIN=D39 MAX=D40
 *
 *   Trash conveyor (per pinmap_mega2560.md):
 *     Stepper STEP / DIR / EN  ->  D4 / D22 / D23
 *     Picker servo             ->  D5
 *     Home limit switch        ->  D34
 *     Left film (bag) sensor   ->  D35
 *     Vacuum sensor            ->  A0
 *     Vacuum pump relay (D24)  ->  NOT USED YET (pump control omitted)
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <AccelStepper.h>

// ============================================================================
//  Pin assignments
// ============================================================================

const int Mixer_shredderGateLeftServoMotor_pin = 6;
const int Mixer_shredderGateRightServoMotor_pin = 7;

const int Mixer_motorController_ENA   = 10;   // ENA on module (Timer 2 PWM)
const int Mixer_motorController_IN1   = 27;   // IN1 on module
const int Mixer_motorController_IN2   = 28;   // IN2 on module

// Trash conveyor (pinmap_mega2560.md).
const int TC_stepperMotorController_step   = 4;
const int TC_stepperMotorController_dir    = 22;
const int TC_stepperMotorController_enable = 23;
const int TC_servoMotor_pin                = 5;
const int TC_vacuumPumpRelay               = 24;
const int TC_stepperHomeLimit              = 34;
const int TC_leftFilmSensor                = 35;
const int TC_rightFilmSensor               = 36;
const int TC_vacuumSensor                  = A0;

// Shredder (pinmap_mega2560.md).
const int Shredder_motorController_onOff     = 25;
const int Shredder_motorController_direction = 26;

// Mixer blast gates - RoboClaw RC-pulse linear actuators (pinmap_mega2560.md).
// Mapping verified on the bench: LEFT  -> RC D9, limits MIN=D37 MAX=D38;
//                               RIGHT -> RC D8, limits MIN=D39 MAX=D40.
const int Mixer_blastGatePin[2]    = { 9, 8 };     // [LEFT, RIGHT] RC pulse out
const int Mixer_blastGateMinPin[2] = { 37, 39 };   // [LEFT, RIGHT] MIN limit
const int Mixer_blastGateMaxPin[2] = { 38, 40 };   // [LEFT, RIGHT] MAX limit
const char* Mixer_blastGateName[2] = { "LEFT", "RIGHT" };
const int BG_LEFT  = 0;
const int BG_RIGHT = 1;

// ============================================================================
//  Constants
// ============================================================================

const int GATE_OPEN_DEG  = 55;
const int GATE_CLOSE_DEG = 0;

const unsigned long ENERGY_PRINT_INTERVAL_MS = 500;

// ── Trash conveyor tunables ────────────────────────────────────────────────
// Picker servo angles (deg) for a 270-degree servo driven by pulse width.
const int TC_servoMinDeg  = 0;
const int TC_servoMaxDeg  = 270;
const int TC_servoMinUs   = 500;
const int TC_servoMaxUs   = 2500;
const int TC_servoUpDeg   = 0;
const int TC_servoDownDeg = 270;

// Vacuum pump relay (D24). This build's pump switch energizes on LOW (verified
// on hardware); set true only for a board/SSR that energizes on HIGH.
const bool TC_pumpRelayActiveHigh = true;

// Shredder motor controller (D25 = ON/OFF, D26 = direction). Adjust these to
// match the drive's logic levels.
const bool ShredderOnOffActiveHigh = true;   // true: HIGH runs the motor
const bool ShredderDirFwdIsHigh    = true;   // true: HIGH = forward

// ── Mixer blast gate (RoboClaw) tunables ───────────────────────────────────
// RC pulse widths: 1500 us neutral; the extremes give full speed. speedPct
// scales the offset from neutral so lower values move slower.
const int BG_STOP_US    = 1500;
const int BG_EXTEND_US  = 1000;   // toward MAX (fully extended) at full speed
const int BG_RETRACT_US = 2000;   // toward MIN (shortest stroke) at full speed
int blastGateSpeedPct   = 100;    // 1-100 % of full speed

const unsigned long BG_MOVE_TIMEOUT_MS   = 8000;   // guard for any single move
const unsigned long BG_DEFAULT_STROKE_MS = 3000;   // assumed travel until cal
const long          BG_POS_DEADBAND_MS   = 40;     // ignore tiny moves
const unsigned long BG_MOVE_TICK_MS      = 5;      // control-loop period

// Vacuum pick detection (sensor on A0).
const float TC_analogReferenceV = 5.0;
const float TC_vacuumThresholdV = 2.5;
const bool  TC_vacuumDetectedWhenVoltageHigh = true;
const int   TC_servoPickStepDeg = 1;
const unsigned long TC_pumpPrimeMs           = 75;
const unsigned long TC_servoStepSettleMs     = 6;
const unsigned long TC_vacuumCheckIntervalMs = 1;
const unsigned long TC_servoReturnMsPerDeg   = 4;
const unsigned long TC_servoReturnMinMs      = 150;
const int TC_vacuumConfirmSamples     = 3;
const unsigned long TC_vacuumConfirmDelayMs = 1;
const int TC_vacuumFastConfirmSamples = 2;

// IR bag-stack sensor (left film sensor). Most 3-pin IR sensors read LOW when
// an object is present.
const bool TC_bagPresentState = LOW;
const int  TC_bagSensorConfirmSamples    = 5;
const int  TC_bagSensorEmptyConfirmCount = 4;
const unsigned long TC_bagSensorConfirmDelayMs = 20;

// Stepper motion: speed in mm/s, accel in mm/s^2.
const float TC_maxSpeed  = 600.0;
const float TC_accel     = 700.0;
const float TC_homeSpeed = 50.0;
const int   TC_minPulseWidthUs = 2;

// 3GT belt with 18T pulley.
const float TC_beltPitchMm = 3.0;
const int   TC_pulleyTeeth = 18;
const int   TC_motorStepsPerRev = 200;
const int   TC_microsteps = 4;

// Positions, in mm from home. Negative moves away from the switch.
const float TC_homePos       = 0.0;
const float TC_bag1Pos       = -25.0;
const float TC_bag2Pos       = -500.0;   // Tune to the Bag 2 stack position.
const float TC_shredderPos   = -475.0;
const float TC_stepperMinPos = -550.0;
const float TC_stepperMaxPos = -10.0;

const int TC_homeDir = 1;   // Change to -1 if homing moves away from the switch.
const unsigned long TC_debounceMs      = 50;
const unsigned long TC_shredderPauseMs = 1000;

// ============================================================================
//  Objects
// ============================================================================

Servo Mixer_shredderGateLeftServoMotor;
Servo Mixer_shredderGateRightServoMotor;
Adafruit_INA219 Mixer_screwMotorCurrentSensor;

AccelStepper TC_stepper(AccelStepper::DRIVER,
                        TC_stepperMotorController_step,
                        TC_stepperMotorController_dir);
Servo TC_servoMotor;

Servo Mixer_blastGateServo[2];

// ============================================================================
//  State
// ============================================================================

bool gateOpen  = false;
int  motorPwm  = 0;
bool motorFwd  = true;   // true = FWD, false = REV
bool inaOk     = false;

String cmdBuffer;
unsigned long lastEnergyPrint = 0;

// Trash conveyor state machine.
enum TCState {
  TC_WAIT_HOME,
  TC_HOMING,
  TC_BACK_TO_BAG1,
  TC_READY,
  TC_MOVE_TO_BAG,
  TC_BAG_TO_SHREDDER,
  TC_MANUAL_MOVE,
  TC_DONE
};

TCState tcState = TC_WAIT_HOME;
int  tcLastLimitRead   = HIGH;
int  tcLimitState      = HIGH;
int  tcBagTripsDone    = 0;
int  tcActiveBag       = 1;
int  tcSequenceStep    = 0;
bool tcSequenceRunning = false;
bool tcIrDetectionEnabled = true;
unsigned long tcLastLimitChange = 0;
int  tcCurrentServoAngle = TC_servoUpDeg;
bool tcPumpRunning     = false;
bool shredderRunning   = false;
bool shredderFwd       = true;

// Mixer blast gate estimated position state (indexed [LEFT, RIGHT]).
unsigned long blastGateStrokeMs[2]   = { BG_DEFAULT_STROKE_MS, BG_DEFAULT_STROKE_MS };
long          blastGatePositionMs[2] = { 0, 0 };   // estimated ms of travel from MIN
bool          blastGateCalibrated[2] = { false, false };
bool          blastGatePrevMin[2]    = { false, false };
bool          blastGatePrevMax[2]    = { false, false };

// ============================================================================
//  Gate helpers
// ============================================================================

void gateOpenCmd() {
  Mixer_shredderGateLeftServoMotor.write(GATE_OPEN_DEG);
  Mixer_shredderGateRightServoMotor.write(GATE_OPEN_DEG);
  gateOpen = true;
  Serial.println(F("[GATE] OPEN"));
}

void gateCloseCmd() {
  Mixer_shredderGateLeftServoMotor.write(GATE_CLOSE_DEG);
  Mixer_shredderGateRightServoMotor.write(GATE_CLOSE_DEG);
  gateOpen = false;
  Serial.println(F("[GATE] CLOSED"));
}

void gateStatus() {
  Serial.print(F("[GATE] state="));
  Serial.println(gateOpen ? F("OPEN") : F("CLOSED"));
}

// ============================================================================
//  Motor helpers
// ============================================================================

void motorApply() {
  if (motorPwm == 0) {
    // Disable ENA first so output transistors are off before touching
    // direction pins — prevents shoot-through on the L298N/TB6612.
    analogWrite(Mixer_motorController_ENA, 0);
    digitalWrite(Mixer_motorController_IN1, LOW);
    digitalWrite(Mixer_motorController_IN2, LOW);
    return;
  }
  // Set direction while ENA is still off, then enable.
  if (motorFwd) {
    digitalWrite(Mixer_motorController_IN1, HIGH);
    digitalWrite(Mixer_motorController_IN2, LOW);
  } else {
    digitalWrite(Mixer_motorController_IN1, LOW);
    digitalWrite(Mixer_motorController_IN2, HIGH);
  }
  analogWrite(Mixer_motorController_ENA, motorPwm);
}

void motorStop() {
  motorPwm = 0;
  motorApply();
  Serial.println(F("[MOTOR] STOPPED"));
}

// MOTOR_TEST: bypass PWM entirely — drives ENA fully HIGH via digitalWrite
// to isolate whether the H-bridge responds at all.
void motorTest() {
  Serial.println(F("[MOTOR_TEST] Setting IN1=HIGH IN2=LOW ENA=HIGH (full on FWD)"));
  digitalWrite(Mixer_motorController_IN1, HIGH);
  digitalWrite(Mixer_motorController_IN2, LOW);
  digitalWrite(Mixer_motorController_ENA, HIGH);
  delay(2000);
  digitalWrite(Mixer_motorController_ENA, LOW);
  Serial.println(F("[MOTOR_TEST] Done. ENA back LOW."));
}

void motorStatus() {
  Serial.print(F("[MOTOR] pwm="));
  Serial.print(motorPwm);
  Serial.print(F(" dir="));
  Serial.println(motorFwd ? F("FWD") : F("REV"));
}

// ============================================================================
//  INA219 helpers
// ============================================================================

void printEnergy() {
  if (!inaOk) {
    Serial.println(F("[ENERGY] unavailable"));
    return;
  }
  float busV     = Mixer_screwMotorCurrentSensor.getBusVoltage_V();
  float shuntMv  = Mixer_screwMotorCurrentSensor.getShuntVoltage_mV();
  float loadV    = busV + (shuntMv / 1000.0f);
  float currentA = Mixer_screwMotorCurrentSensor.getCurrent_mA() / 1000.0f;
  float powerW   = Mixer_screwMotorCurrentSensor.getPower_mW()   / 1000.0f;

  Serial.print(F("[ENERGY] pwm="));
  Serial.print(motorPwm);
  Serial.print(F(" dir="));
  Serial.print(motorFwd ? F("FWD") : F("REV"));
  Serial.print(F(" V="));
  Serial.print(loadV, 2);
  Serial.print(F(" I="));
  Serial.print(currentA, 3);
  Serial.print(F(" P="));
  Serial.println(powerW, 2);
}

// ============================================================================
//  Trash Conveyor - Pick & Place
//  Ported from Trash_Conveyor-Test-Pick_Place_Sequence. Pins follow
//  pinmap_mega2560.md. TC_PICK runs the repeating sequence: Bag 1 four times,
//  then Bag 2 once, and repeat. The vacuum pump (relay D24) is primed before
//  each pick; a missing bag or lost vacuum stops the sequence.
// ============================================================================

long tcMmToSteps(float mm) {
  float mmPerRev   = TC_beltPitchMm * TC_pulleyTeeth;
  float stepsPerMm = (TC_motorStepsPerRev * TC_microsteps) / mmPerRev;
  return lround(mm * stepsPerMm);
}

float tcStepsToMm(long steps) {
  float mmPerRev   = TC_beltPitchMm * TC_pulleyTeeth;
  float stepsPerMm = (TC_motorStepsPerRev * TC_microsteps) / mmPerRev;
  return steps / stepsPerMm;
}

void tcReadLimitSwitch() {
  int reading = digitalRead(TC_stepperHomeLimit);
  if (reading != tcLastLimitRead) {
    tcLastLimitChange = millis();
  }
  if (millis() - tcLastLimitChange > TC_debounceMs) {
    tcLimitState = reading;
  }
  tcLastLimitRead = reading;
}

bool tcLimitPressed() {
  return tcLimitState == LOW;
}

float tcRawToVoltage(int raw) {
  return raw * (TC_analogReferenceV / 1023.0);
}

bool tcVacuumDetectedOnce() {
  int raw = analogRead(TC_vacuumSensor);
  float voltage = tcRawToVoltage(raw);
  return TC_vacuumDetectedWhenVoltageHigh
           ? voltage >= TC_vacuumThresholdV
           : voltage <= TC_vacuumThresholdV;
}

bool tcVacuumDetectedDuringSettle() {
  unsigned long settleStart = millis();
  int consecutive = 0;
  while (millis() - settleStart < TC_servoStepSettleMs) {
    if (tcVacuumDetectedOnce()) {
      consecutive++;
    } else {
      consecutive = 0;
    }
    if (consecutive >= TC_vacuumFastConfirmSamples) {
      return true;
    }
    delay(TC_vacuumCheckIntervalMs);
  }
  return false;
}

int tcActiveBagSensorPin() {
  return tcActiveBag == 2 ? TC_rightFilmSensor : TC_leftFilmSensor;
}

bool tcBagStackEmptyConfirmed() {
  if (!tcIrDetectionEnabled) {
    return false;
  }
  int emptySamples = 0;
  for (int i = 0; i < TC_bagSensorConfirmSamples; i++) {
    bool bagPresent = digitalRead(tcActiveBagSensorPin()) == TC_bagPresentState;
    if (!bagPresent) {
      emptySamples++;
    }
    if (i < TC_bagSensorConfirmSamples - 1) {
      delay(TC_bagSensorConfirmDelayMs);
    }
  }
  return emptySamples >= TC_bagSensorEmptyConfirmCount;
}

void tcWriteServoAngle(int angle) {
  tcCurrentServoAngle = constrain(angle, TC_servoMinDeg, TC_servoMaxDeg);
  int pulseWidth = map(tcCurrentServoAngle, TC_servoMinDeg, TC_servoMaxDeg, TC_servoMinUs, TC_servoMaxUs);
  TC_servoMotor.writeMicroseconds(pulseWidth);
}

void tcPumpOn() {
  digitalWrite(TC_vacuumPumpRelay, TC_pumpRelayActiveHigh ? HIGH : LOW);
  tcPumpRunning = true;
}

void tcPumpOff() {
  digitalWrite(TC_vacuumPumpRelay, TC_pumpRelayActiveHigh ? LOW : HIGH);
  tcPumpRunning = false;
}

void tcSetIrDetection(bool enabled) {
  tcIrDetectionEnabled = enabled;
  Serial.print(F("[TC] IR detection "));
  Serial.println(enabled ? F("ENABLED") : F("DISABLED"));
}

void tcServoUp() {
  tcWriteServoAngle(TC_servoUpDeg);
}

void tcServoDown() {
  tcWriteServoAngle(TC_servoDownDeg);
}

void tcReturnServoToUp() {
  int returnDistance = abs(tcCurrentServoAngle - TC_servoUpDeg);
  tcWriteServoAngle(TC_servoUpDeg);
  delay(max(TC_servoReturnMinMs, (unsigned long)(returnDistance * TC_servoReturnMsPerDeg)));
}

void tcMoveTo(float targetMm, TCState nextState) {
  TC_stepper.enableOutputs();
  TC_stepper.moveTo(tcMmToSteps(targetMm));
  tcState = nextState;
}

void tcStopPickSequence(const __FlashStringHelper* message) {
  tcSequenceRunning = false;
  tcPumpOff();
  TC_stepper.stop();
  TC_stepper.disableOutputs();
  tcState = TC_DONE;
  Serial.println(message);
}

int tcNextSequenceBag() {
  // Pick Bag 1 four times, then Bag 2 once, then repeat.
  return tcSequenceStep < 4 ? 1 : 2;
}

void tcAdvanceSequenceStep() {
  tcSequenceStep++;
  if (tcSequenceStep >= 5) {
    tcSequenceStep = 0;
  }
}

float tcActiveBagPosition() {
  return tcActiveBag == 2 ? TC_bag2Pos : TC_bag1Pos;
}

const __FlashStringHelper* tcActiveBagName() {
  return tcActiveBag == 2 ? F("Bag 2") : F("Bag 1");
}

void tcDropAtShredderAndContinue() {
  // Release the bag over the shredder, then dwell.
  tcPumpOff();
  delay(TC_shredderPauseMs);

  tcBagTripsDone++;
  Serial.print(F("[TC] Dropped "));
  Serial.print(tcActiveBagName());
  Serial.print(F(" at shredder. Trips: "));
  Serial.println(tcBagTripsDone);

  tcAdvanceSequenceStep();

  if (!tcSequenceRunning) {
    TC_stepper.disableOutputs();
    tcState = TC_READY;
    Serial.println(F("[TC] Sequence paused after shredder drop"));
    return;
  }

  tcActiveBag = tcNextSequenceBag();
  tcMoveTo(tcActiveBagPosition(), TC_MOVE_TO_BAG);
  Serial.print(F("[TC] Next pick: "));
  Serial.print(tcActiveBagName());
  Serial.print(F(" | step "));
  Serial.print(tcSequenceStep + 1);
  Serial.println(F(" of 5"));
}

bool tcPickAtBag() {
  if (tcBagStackEmptyConfirmed()) {
    tcStopPickSequence(F("[TC] Bag not detected - sequence stopped"));
    return false;
  }

  // Prime the vacuum pump, then descend; grab as soon as the cup seals.
  tcPumpOn();
  delay(TC_pumpPrimeMs);

  bool bagGrabbed = false;
  for (int angle = TC_servoUpDeg; angle <= TC_servoDownDeg; angle += TC_servoPickStepDeg) {
    tcWriteServoAngle(angle);
    if (tcVacuumDetectedDuringSettle()) {
      bagGrabbed = true;
      Serial.print(F("[TC] Vacuum detected at servo "));
      Serial.print(tcCurrentServoAngle);
      Serial.println(F(" deg"));
      break;
    }
  }

  if (!bagGrabbed) {
    tcServoDown();
    delay(TC_servoStepSettleMs);
    tcStopPickSequence(F("[TC] Vacuum not detected - sequence stopped"));
    return false;
  }

  tcReturnServoToUp();
  return true;
}

void tcStartHome() {
  tcSequenceRunning = false;
  tcBagTripsDone = 0;
  tcActiveBag = 1;
  tcSequenceStep = 0;
  tcState = TC_HOMING;
  TC_stepper.enableOutputs();
  TC_stepper.setSpeed(tcMmToSteps(TC_homeSpeed) * TC_homeDir);
  Serial.println(F("[TC] Homing..."));
}

void tcRunHome() {
  if (tcLimitPressed()) {
    TC_stepper.setCurrentPosition(tcMmToSteps(TC_homePos));
    TC_stepper.setSpeed(0);
    tcMoveTo(TC_bag1Pos, TC_BACK_TO_BAG1);
    Serial.println(F("[TC] Limit hit - moving to Bag 1"));
    return;
  }
  TC_stepper.runSpeed();
}

void tcStartPick() {
  if (tcState != TC_READY && tcState != TC_DONE) {
    Serial.println(F("[TC] Home first (TC_HOME) and wait until ready"));
    return;
  }
  tcSequenceRunning = true;
  tcBagTripsDone = 0;
  tcSequenceStep = 0;
  tcActiveBag = tcNextSequenceBag();
  tcMoveTo(tcActiveBagPosition(), TC_MOVE_TO_BAG);
  Serial.print(F("[TC] Sequence started - moving to "));
  Serial.println(tcActiveBagName());
}

void tcMoveStepperTo(float targetMm) {
  if (tcState == TC_WAIT_HOME || tcState == TC_HOMING) {
    Serial.println(F("[TC] Home first before a manual move"));
    return;
  }
  if (tcState != TC_READY && tcState != TC_DONE && tcState != TC_MANUAL_MOVE) {
    Serial.println(F("[TC] Busy - wait for the current move to finish"));
    return;
  }
  targetMm = constrain(targetMm, TC_stepperMinPos, TC_stepperMaxPos);
  tcSequenceRunning = false;
  tcMoveTo(targetMm, TC_MANUAL_MOVE);
  Serial.print(F("[TC] Moving to "));
  Serial.print(targetMm);
  Serial.println(F(" mm"));
}

void tcRunState() {
  if (tcState == TC_HOMING) {
    tcRunHome();
    return;
  }

  TC_stepper.run();

  if (TC_stepper.distanceToGo() != 0) {
    return;
  }

  switch (tcState) {
    case TC_BACK_TO_BAG1:
      TC_stepper.disableOutputs();
      tcState = TC_READY;
      Serial.println(F("[TC] Homed - at Bag 1, send TC_PICK"));
      break;

    case TC_MOVE_TO_BAG:
      if (tcPickAtBag()) {
        tcMoveTo(TC_shredderPos, TC_BAG_TO_SHREDDER);
        Serial.println(F("[TC] Pick complete - moving to shredder"));
      }
      break;

    case TC_BAG_TO_SHREDDER:
      tcDropAtShredderAndContinue();
      break;

    case TC_MANUAL_MOVE:
      TC_stepper.disableOutputs();
      tcState = TC_READY;
      Serial.println(F("[TC] Manual move complete"));
      break;

    default:
      break;
  }
}

void tcUpdate() {
  tcReadLimitSwitch();
  tcRunState();
}

const __FlashStringHelper* tcStateName() {
  switch (tcState) {
    case TC_HOMING:
    case TC_BACK_TO_BAG1:     return F("HOMING");
    case TC_READY:            return F("READY");
    case TC_MOVE_TO_BAG:
    case TC_BAG_TO_SHREDDER:  return F("PICKING");
    case TC_MANUAL_MOVE:      return F("MANUAL");
    case TC_DONE:             return F("DONE");
    default:                  return F("WAIT_HOME");
  }
}

void tcPrintStatus() {
  Serial.print(F("[TC] state="));
  Serial.print(tcStateName());
  Serial.print(F(" pos_mm="));
  Serial.print(tcStepsToMm(TC_stepper.currentPosition()), 1);
  Serial.print(F(" limit="));
  Serial.print(tcLimitPressed() ? F("PRESSED") : F("OPEN"));
  Serial.print(F(" bag_num="));
  Serial.print(tcActiveBag);
  Serial.print(F(" bag="));
  if (tcIrDetectionEnabled) {
    Serial.print(tcBagStackEmptyConfirmed() ? F("EMPTY") : F("DETECTED"));
  } else {
    Serial.print(F("IR_OFF"));
  }
  int vacuumRaw = analogRead(TC_vacuumSensor);
  Serial.print(F(" pump="));
  Serial.print(tcPumpRunning ? F("ON") : F("OFF"));
  Serial.print(F(" vac="));
  Serial.print(tcVacuumDetectedOnce() ? F("YES") : F("NO"));
  Serial.print(F(" vac_v="));
  Serial.print(tcRawToVoltage(vacuumRaw), 2);
  Serial.print(F(" step="));
  Serial.print(tcSequenceStep + 1);
  Serial.print(F(" seq="));
  Serial.print(tcSequenceRunning ? F("RUN") : F("IDLE"));
  Serial.print(F(" trips="));
  Serial.print(tcBagTripsDone);
  Serial.print(F(" servo="));
  Serial.println(tcCurrentServoAngle);
}

// ============================================================================
//  Shredder
// ============================================================================

void shredderApply() {
  bool dirLevel = ShredderDirFwdIsHigh ? shredderFwd : !shredderFwd;
  bool onLevel  = ShredderOnOffActiveHigh ? shredderRunning : !shredderRunning;
  // Set direction before enabling so the drive sees a stable dir signal.
  digitalWrite(Shredder_motorController_direction, dirLevel ? HIGH : LOW);
  digitalWrite(Shredder_motorController_onOff, onLevel ? HIGH : LOW);
}

void shredderOn() {
  shredderRunning = true;
  shredderApply();
}

void shredderOff() {
  shredderRunning = false;
  shredderApply();
}

void shredderSetDirection(bool fwd) {
  shredderFwd = fwd;
  shredderApply();
}

// ============================================================================
//  Mixer Blast Gates - RoboClaw linear actuators
//  Ported from Mixer-Test-BlastGate_PositionUtility. Position is ESTIMATED
//  from timed motion (no encoders); the MIN / MAX limit switches provide the
//  0% / 100% references. Moves are blocking; any serial byte aborts a move.
// ============================================================================

bool bgMinHit(int g) { return digitalRead(Mixer_blastGateMinPin[g]) == LOW; }
bool bgMaxHit(int g) { return digitalRead(Mixer_blastGateMaxPin[g]) == LOW; }

void bgStop(int g) { Mixer_blastGateServo[g].writeMicroseconds(BG_STOP_US); }

void bgExtend(int g) {
  int us = BG_STOP_US + (int)((long)(BG_EXTEND_US - BG_STOP_US) * blastGateSpeedPct / 100);
  Mixer_blastGateServo[g].writeMicroseconds(us);
}

void bgRetract(int g) {
  int us = BG_STOP_US + (int)((long)(BG_RETRACT_US - BG_STOP_US) * blastGateSpeedPct / 100);
  Mixer_blastGateServo[g].writeMicroseconds(us);
}

void bgStopAll() {
  bgStop(BG_LEFT);
  bgStop(BG_RIGHT);
}

// True if the user sent any byte (used to abort a blocking move).
bool bgAbortRequested() {
  if (Serial.available() > 0) {
    while (Serial.available() > 0) Serial.read();  // flush
    return true;
  }
  return false;
}

// Watch all four limit switches and report press / release transitions.
void bgMonitorLimits() {
  for (int g = 0; g < 2; g++) {
    bool m = bgMinHit(g);
    if (m != blastGatePrevMin[g]) {
      Serial.print(F("[BLASTGATE "));
      Serial.print(Mixer_blastGateName[g]);
      Serial.println(m ? F("] MIN limit PRESSED") : F("] MIN limit released"));
      blastGatePrevMin[g] = m;
    }
    bool x = bgMaxHit(g);
    if (x != blastGatePrevMax[g]) {
      Serial.print(F("[BLASTGATE "));
      Serial.print(Mixer_blastGateName[g]);
      Serial.println(x ? F("] MAX limit PRESSED") : F("] MAX limit released"));
      blastGatePrevMax[g] = x;
    }
  }
}

float bgPercent(int g) {
  if (blastGateStrokeMs[g] == 0) return 0.0;
  return 100.0 * (float)blastGatePositionMs[g] / (float)blastGateStrokeMs[g];
}

bool bgHome(int g) {
  Serial.print(F("[BLASTGATE "));
  Serial.print(Mixer_blastGateName[g]);
  Serial.println(F("] Homing to MIN..."));

  unsigned long start = millis();
  while (!bgMinHit(g)) {
    bgMonitorLimits();
    if (bgAbortRequested()) { bgStop(g); Serial.println(F("[BLASTGATE] aborted")); return false; }
    bgRetract(g);
    if (millis() - start > BG_MOVE_TIMEOUT_MS) {
      bgStop(g);
      Serial.println(F("[BLASTGATE] MIN timeout - check wiring/stroke time"));
      return false;
    }
    delay(BG_MOVE_TICK_MS);
  }
  bgStop(g);
  blastGatePositionMs[g] = 0;
  Serial.print(F("[BLASTGATE "));
  Serial.print(Mixer_blastGateName[g]);
  Serial.println(F("] at MIN (0%)"));
  return true;
}

bool bgHomeMax(int g) {
  Serial.print(F("[BLASTGATE "));
  Serial.print(Mixer_blastGateName[g]);
  Serial.println(F("] Homing to MAX..."));

  unsigned long start = millis();
  while (!bgMaxHit(g)) {
    bgMonitorLimits();
    if (bgAbortRequested()) { bgStop(g); Serial.println(F("[BLASTGATE] aborted")); return false; }
    bgExtend(g);
    if (millis() - start > BG_MOVE_TIMEOUT_MS) {
      bgStop(g);
      Serial.println(F("[BLASTGATE] MAX timeout - check wiring/stroke time"));
      return false;
    }
    delay(BG_MOVE_TICK_MS);
  }
  bgStop(g);
  blastGatePositionMs[g] = blastGateStrokeMs[g];
  Serial.print(F("[BLASTGATE "));
  Serial.print(Mixer_blastGateName[g]);
  Serial.println(F("] at MAX (100%)"));
  return true;
}

bool bgCalibrate(int g) {
  if (!bgHome(g)) return false;

  Serial.print(F("[BLASTGATE "));
  Serial.print(Mixer_blastGateName[g]);
  Serial.println(F("] Calibrating - timing MIN->MAX..."));

  unsigned long start = millis();
  while (!bgMaxHit(g)) {
    bgMonitorLimits();
    if (bgAbortRequested()) { bgStop(g); Serial.println(F("[BLASTGATE] aborted")); return false; }
    bgExtend(g);
    if (millis() - start > BG_MOVE_TIMEOUT_MS) {
      bgStop(g);
      Serial.println(F("[BLASTGATE] MAX timeout - calibration failed"));
      return false;
    }
    delay(BG_MOVE_TICK_MS);
  }
  bgStop(g);

  blastGateStrokeMs[g]   = millis() - start;
  blastGatePositionMs[g] = blastGateStrokeMs[g];
  blastGateCalibrated[g] = true;

  Serial.print(F("[BLASTGATE "));
  Serial.print(Mixer_blastGateName[g]);
  Serial.print(F("] stroke="));
  Serial.print(blastGateStrokeMs[g]);
  Serial.println(F(" ms (at MAX / 100%)"));
  return true;
}

void bgMoveToPercent(int g, float pct) {
  pct = constrain(pct, 0.0, 100.0);

  if (!blastGateCalibrated[g]) {
    Serial.print(F("[BLASTGATE "));
    Serial.print(Mixer_blastGateName[g]);
    Serial.print(F("] WARNING not calibrated - assumed stroke "));
    Serial.print(blastGateStrokeMs[g]);
    Serial.println(F(" ms; run BLASTGATE_CAL"));
  }

  long targetMs = (long)((float)blastGateStrokeMs[g] * pct / 100.0);
  long deltaMs  = targetMs - blastGatePositionMs[g];

  if (labs(deltaMs) < BG_POS_DEADBAND_MS) {
    Serial.print(F("[BLASTGATE "));
    Serial.print(Mixer_blastGateName[g]);
    Serial.println(F("] already at target"));
    return;
  }

  bool extending = deltaMs > 0;
  unsigned long moveTime = (unsigned long)labs(deltaMs);

  Serial.print(F("[BLASTGATE "));
  Serial.print(Mixer_blastGateName[g]);
  Serial.print(F("] moving to "));
  Serial.print(pct, 1);
  Serial.print(F("% ("));
  Serial.print(extending ? F("extend ") : F("retract "));
  Serial.print(moveTime);
  Serial.println(F(" ms)"));

  unsigned long start = millis();
  bool hitLimit = false;
  while (millis() - start < moveTime) {
    bgMonitorLimits();
    if (bgAbortRequested()) { Serial.println(F("[BLASTGATE] aborted")); break; }
    if (extending) {
      if (bgMaxHit(g)) { hitLimit = true; break; }
      bgExtend(g);
    } else {
      if (bgMinHit(g)) { hitLimit = true; break; }
      bgRetract(g);
    }
    if (millis() - start > BG_MOVE_TIMEOUT_MS) { Serial.println(F("[BLASTGATE] move timeout")); break; }
    delay(BG_MOVE_TICK_MS);
  }
  bgStop(g);

  unsigned long elapsed = millis() - start;
  blastGatePositionMs[g] += extending ? (long)elapsed : -(long)elapsed;
  if (bgMinHit(g)) blastGatePositionMs[g] = 0;
  if (bgMaxHit(g)) blastGatePositionMs[g] = blastGateStrokeMs[g];
  blastGatePositionMs[g] = constrain(blastGatePositionMs[g], 0L, (long)blastGateStrokeMs[g]);

  Serial.print(F("[BLASTGATE "));
  Serial.print(Mixer_blastGateName[g]);
  Serial.print(F("] done at ~"));
  Serial.print(bgPercent(g), 1);
  Serial.print(F("%"));
  if (hitLimit) Serial.print(F(" (limit)"));
  Serial.println();
}

void bgJog(int g, bool extend, unsigned long ms) {
  Serial.print(F("[BLASTGATE "));
  Serial.print(Mixer_blastGateName[g]);
  Serial.print(extend ? F("] extending ") : F("] retracting "));
  Serial.print(ms);
  Serial.println(F(" ms"));

  unsigned long start = millis();
  while (millis() - start < ms) {
    bgMonitorLimits();
    if (bgAbortRequested()) { Serial.println(F("[BLASTGATE] aborted")); break; }
    if (extend) {
      if (bgMaxHit(g)) { Serial.println(F("[BLASTGATE] MAX limit")); break; }
      bgExtend(g);
    } else {
      if (bgMinHit(g)) { Serial.println(F("[BLASTGATE] MIN limit")); break; }
      bgRetract(g);
    }
    delay(BG_MOVE_TICK_MS);
  }
  bgStop(g);

  unsigned long elapsed = millis() - start;
  blastGatePositionMs[g] += extend ? (long)elapsed : -(long)elapsed;
  if (bgMinHit(g)) blastGatePositionMs[g] = 0;
  if (bgMaxHit(g)) blastGatePositionMs[g] = blastGateStrokeMs[g];
  blastGatePositionMs[g] = constrain(blastGatePositionMs[g], 0L, (long)blastGateStrokeMs[g]);
}

void bgPrintStatus() {
  for (int g = 0; g < 2; g++) {
    Serial.print(F("[BLASTGATE "));
    Serial.print(Mixer_blastGateName[g]);
    Serial.print(F("] pos=~"));
    Serial.print(bgPercent(g), 1);
    Serial.print(F("% stroke="));
    Serial.print(blastGateStrokeMs[g]);
    Serial.print(F(" ms cal="));
    Serial.print(blastGateCalibrated[g] ? F("YES") : F("NO"));
    Serial.print(F(" MIN="));
    Serial.print(bgMinHit(g) ? F("HIT") : F("open"));
    Serial.print(F(" MAX="));
    Serial.println(bgMaxHit(g) ? F("HIT") : F("open"));
  }
}

int bgParseGate(const String& token) {
  String t = token; t.trim(); t.toUpperCase();
  if (t == "L" || t == "LEFT")  return BG_LEFT;
  if (t == "R" || t == "RIGHT") return BG_RIGHT;
  return -1;
}

void bgParseSelection(const String& sel, bool& doL, bool& doR) {
  String g = sel; g.trim(); g.toUpperCase();
  doL = (g == "ALL" || g == "L" || g == "LEFT");
  doR = (g == "ALL" || g == "R" || g == "RIGHT");
}

// Dispatch a single BLASTGATE_* command. The caller prints [BLASTGATE_DONE]
// afterward so the host knows the (possibly multi-second, blocking) op finished.
void handleBlastGateCommand(const String& cmd) {
  if (cmd.startsWith("BLASTGATE_HOMEMAX ")) {
    bool doL, doR; bgParseSelection(cmd.substring(18), doL, doR);
    if (!doL && !doR) { Serial.println(F("[BLASTGATE] usage BLASTGATE_HOMEMAX <L|R|ALL>")); return; }
    if (doL) bgHomeMax(BG_LEFT);
    if (doR) bgHomeMax(BG_RIGHT);

  } else if (cmd.startsWith("BLASTGATE_HOME ")) {
    bool doL, doR; bgParseSelection(cmd.substring(15), doL, doR);
    if (!doL && !doR) { Serial.println(F("[BLASTGATE] usage BLASTGATE_HOME <L|R|ALL>")); return; }
    if (doL) bgHome(BG_LEFT);
    if (doR) bgHome(BG_RIGHT);

  } else if (cmd.startsWith("BLASTGATE_CAL ")) {
    bool doL, doR; bgParseSelection(cmd.substring(14), doL, doR);
    if (!doL && !doR) { Serial.println(F("[BLASTGATE] usage BLASTGATE_CAL <L|R|ALL>")); return; }
    if (doL) bgCalibrate(BG_LEFT);
    if (doR) bgCalibrate(BG_RIGHT);

  } else if (cmd.startsWith("BLASTGATE_POS ")) {
    String args = cmd.substring(14); args.trim();
    int sp = args.indexOf(' ');
    int g = (sp < 0) ? -1 : bgParseGate(args.substring(0, sp));
    if (g < 0 || sp < 0) { Serial.println(F("[BLASTGATE] usage BLASTGATE_POS <L|R> <0-100>")); return; }
    bgMoveToPercent(g, args.substring(sp + 1).toFloat());

  } else if (cmd.startsWith("BLASTGATE_EXT ") || cmd.startsWith("BLASTGATE_RET ")) {
    bool ext = cmd.startsWith("BLASTGATE_EXT ");
    String args = cmd.substring(14); args.trim();
    int sp = args.indexOf(' ');
    int g = (sp < 0) ? -1 : bgParseGate(args.substring(0, sp));
    long ms = (sp < 0) ? 0 : args.substring(sp + 1).toInt();
    if (g < 0 || ms <= 0) { Serial.println(F("[BLASTGATE] usage BLASTGATE_EXT/RET <L|R> <ms>")); return; }
    bgJog(g, ext, (unsigned long)ms);

  } else if (cmd.startsWith("BLASTGATE_SPEED ")) {
    int v = cmd.substring(16).toInt();
    if (v < 1 || v > 100) { Serial.println(F("[BLASTGATE] speed must be 1-100")); return; }
    blastGateSpeedPct = v;
    Serial.print(F("[BLASTGATE] speed="));
    Serial.print(blastGateSpeedPct);
    Serial.println(F("%"));

  } else if (cmd == "BLASTGATE_STOP") {
    bgStopAll();
    Serial.println(F("[BLASTGATE] stopped both"));

  } else if (cmd == "BLASTGATE_STATUS") {
    bgPrintStatus();

  } else {
    Serial.print(F("[BLASTGATE] unknown: "));
    Serial.println(cmd);
  }
}

// ============================================================================
//  Global STATUS
// ============================================================================

void printAllStatus() {
  Serial.print(F("[STATUS] gate="));
  Serial.print(gateOpen ? F("OPEN") : F("CLOSED"));
  Serial.print(F(" motor_pwm="));
  Serial.print(motorPwm);
  Serial.print(F(" motor_dir="));
  Serial.print(motorFwd ? F("FWD") : F("REV"));
  Serial.print(F(" tc_state="));
  Serial.print(tcStateName());
  Serial.print(F(" tc_pos_mm="));
  Serial.print(tcStepsToMm(TC_stepper.currentPosition()), 1);
  Serial.print(F(" tc_bag_num="));
  Serial.print(tcActiveBag);
  Serial.print(F(" tc_bag="));
  Serial.print((digitalRead(tcActiveBagSensorPin()) == TC_bagPresentState) ? F("DETECTED") : F("EMPTY"));
  Serial.print(F(" tc_pump="));
  Serial.print(tcPumpRunning ? F("ON") : F("OFF"));
  Serial.print(F(" shredder="));
  Serial.print(shredderRunning ? F("ON") : F("OFF"));
  Serial.print(F(" shredder_dir="));
  Serial.print(shredderFwd ? F("FWD") : F("REV"));
  Serial.print(F(" bg_left="));
  Serial.print(bgPercent(BG_LEFT), 0);
  Serial.print(F(" bg_right="));
  Serial.println(bgPercent(BG_RIGHT), 0);
  printEnergy();
}

// ============================================================================
//  Command dispatch
// ============================================================================

void handleCommand(const String& cmd) {
  if (cmd == "GATE_OPEN") {
    gateOpenCmd();

  } else if (cmd == "GATE_CLOSE") {
    gateCloseCmd();

  } else if (cmd == "GATE_STATUS") {
    gateStatus();

  } else if (cmd.startsWith("MOTOR_SET ")) {
    // MOTOR_SET <0-255> <FWD|REV>
    String args = cmd.substring(10);
    args.trim();
    int spaceIdx = args.indexOf(' ');
    if (spaceIdx < 0) {
      Serial.println(F("[MOTOR] ERROR: usage MOTOR_SET <0-255> <FWD|REV>"));
      return;
    }
    int spd    = args.substring(0, spaceIdx).toInt();
    String dir = args.substring(spaceIdx + 1);
    dir.trim();

    if (spd < 0 || spd > 255) {
      Serial.println(F("[MOTOR] ERROR: speed must be 0-255"));
      return;
    }
    if (dir != "FWD" && dir != "REV") {
      Serial.println(F("[MOTOR] ERROR: direction must be FWD or REV"));
      return;
    }

    motorPwm = spd;
    motorFwd = (dir == "FWD");
    motorApply();
    motorStatus();

  } else if (cmd == "MOTOR_STOP") {
    motorStop();

  } else if (cmd == "MOTOR_STATUS") {
    motorStatus();

  } else if (cmd == "MOTOR_TEST") {
    motorTest();

  } else if (cmd == "TC_HOME") {
    tcStartHome();

  } else if (cmd == "TC_PICK") {
    tcStartPick();

  } else if (cmd == "TC_STOP") {
    tcStopPickSequence(F("[TC] Pick sequence stopped by user"));

  } else if (cmd == "TC_UP") {
    tcServoUp();
    Serial.println(F("[TC] Servo up"));

  } else if (cmd == "TC_DOWN") {
    tcServoDown();
    Serial.println(F("[TC] Servo down"));

  } else if (cmd == "TC_PUMP_ON") {
    tcPumpOn();
    Serial.println(F("[TC] Vacuum pump ON"));

  } else if (cmd == "TC_PUMP_OFF") {
    tcPumpOff();
    Serial.println(F("[TC] Vacuum pump OFF"));

  } else if (cmd == "SHREDDER_ON") {
    shredderOn();
    Serial.print(F("[SHREDDER] Shredder ON "));
    Serial.println(shredderFwd ? F("FWD") : F("REV"));

  } else if (cmd == "SHREDDER_OFF") {
    shredderOff();
    Serial.println(F("[SHREDDER] Shredder OFF"));

  } else if (cmd == "SHREDDER_FWD") {
    shredderSetDirection(true);
    Serial.println(F("[SHREDDER] Direction FWD"));

  } else if (cmd == "SHREDDER_REV") {
    shredderSetDirection(false);
    Serial.println(F("[SHREDDER] Direction REV"));

  } else if (cmd.startsWith("BLASTGATE_")) {
    handleBlastGateCommand(cmd);
    Serial.println(F("[BLASTGATE_DONE]"));   // unique terminal marker for the host

  } else if (cmd == "TC_IR_ON") {
    tcSetIrDetection(true);

  } else if (cmd == "TC_IR_OFF") {
    tcSetIrDetection(false);

  } else if (cmd.startsWith("TC_SERVO ")) {
    int deg = cmd.substring(9).toInt();
    if (deg < TC_servoMinDeg || deg > TC_servoMaxDeg) {
      Serial.println(F("[TC] ERROR: servo angle must be 0-270"));
    } else {
      tcWriteServoAngle(deg);
      Serial.print(F("[TC] Servo "));
      Serial.println(tcCurrentServoAngle);
    }

  } else if (cmd.startsWith("TC_MOVE ")) {
    tcMoveStepperTo(cmd.substring(8).toFloat());

  } else if (cmd == "TC_STATUS") {
    tcPrintStatus();

  } else if (cmd == "STATUS") {
    printAllStatus();

  } else if (cmd == "ESTOP") {
    motorStop();
    gateCloseCmd();
    shredderOff();
    bgStopAll();
    tcStopPickSequence(F("[TC] ESTOP - conveyor halted"));
    tcState = TC_WAIT_HOME;
    Serial.println(F("[SYSTEM] ESTOP - all stopped"));

  } else if (cmd.length() > 0) {
    Serial.print(F("[SYSTEM] Unknown: "));
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

// ============================================================================
//  Setup / Loop
// ============================================================================

void setup() {
  Serial.begin(9600);

  // Gate servos - start closed
  Mixer_shredderGateLeftServoMotor.attach(Mixer_shredderGateLeftServoMotor_pin);
  Mixer_shredderGateRightServoMotor.attach(Mixer_shredderGateRightServoMotor_pin);
  Mixer_shredderGateLeftServoMotor.write(GATE_CLOSE_DEG);
  Mixer_shredderGateRightServoMotor.write(GATE_CLOSE_DEG);

  // Motor driver - start stopped
  pinMode(Mixer_motorController_ENA, OUTPUT);
  pinMode(Mixer_motorController_IN1, OUTPUT);
  pinMode(Mixer_motorController_IN2, OUTPUT);
  motorStop();

  // INA219
  Wire.begin();
  inaOk = Mixer_screwMotorCurrentSensor.begin();
  if (!inaOk) {
    Serial.println(F("[SYSTEM] WARNING: INA219 not found - energy monitor disabled"));
  }

  // Shredder motor controller - off, forward at boot
  pinMode(Shredder_motorController_onOff, OUTPUT);
  pinMode(Shredder_motorController_direction, OUTPUT);
  shredderOff();

  // Mixer blast gates - hold RC neutral (1500 us) so the RoboClaw arms;
  // limit switches use internal pull-ups (active LOW).
  for (int g = 0; g < 2; g++) {
    pinMode(Mixer_blastGateMinPin[g], INPUT_PULLUP);
    pinMode(Mixer_blastGateMaxPin[g], INPUT_PULLUP);
    Mixer_blastGateServo[g].attach(Mixer_blastGatePin[g]);
  }
  bgStopAll();

  // Trash conveyor - servo up, pump off, stepper disabled, awaiting TC_HOME
  pinMode(TC_stepperHomeLimit, INPUT_PULLUP);
  pinMode(TC_leftFilmSensor, INPUT);
  pinMode(TC_rightFilmSensor, INPUT);
  pinMode(TC_vacuumPumpRelay, OUTPUT);
  tcPumpOff();
  TC_servoMotor.attach(TC_servoMotor_pin, TC_servoMinUs, TC_servoMaxUs);
  tcServoUp();
  TC_stepper.setEnablePin(TC_stepperMotorController_enable);
  TC_stepper.setPinsInverted(false, false, true);  // Enable pin is active LOW
  TC_stepper.setMinPulseWidth(TC_minPulseWidthUs);
  TC_stepper.setMaxSpeed(tcMmToSteps(TC_maxSpeed));
  TC_stepper.setAcceleration(tcMmToSteps(TC_accel));
  TC_stepper.disableOutputs();

  Serial.println(F("[SYSTEM] LunaRecycle Mega firmware ready"));
  Serial.println(F("[SYSTEM] Commands: GATE_OPEN, GATE_CLOSE, MOTOR_SET <spd> <FWD|REV>, MOTOR_STOP, TC_HOME, TC_PICK, TC_STOP, STATUS, ESTOP"));
}

void loop() {
  readSerial();

  // Drive the trash-conveyor stepper / pick-place state machine
  tcUpdate();

  // Stream energy readings automatically every 500 ms
  unsigned long now = millis();
  if (now - lastEnergyPrint >= ENERGY_PRINT_INTERVAL_MS) {
    lastEnergyPrint = now;
    printEnergy();
  }
}

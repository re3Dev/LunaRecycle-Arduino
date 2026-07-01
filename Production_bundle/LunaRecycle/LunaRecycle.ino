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
 *   TC_PICK                Run the pick-and-place cycle (Bag 1 -> shredder)
 *   TC_STOP                Stop the pick cycle
 *   TC_UP / TC_DOWN        Raise / lower the picker servo
 *   TC_SERVO <0-180>       Move picker servo to an angle (deg)
 *   TC_PUMP_ON / _OFF      Manually switch the vacuum pump relay
 *   TC_MOVE <-550..-10>    Manual stepper move (mm from home)
 *   TC_STATUS              Print trash-conveyor detail status
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

const int Mixer_shredderGateLeftServoMotor_pin = 9;
const int Mixer_shredderGateRightServoMotor_pin = 10;

const int Mixer_motorController_ENA   = 3;   // ENA on module (Timer 2 PWM)
const int Mixer_motorController_IN1   = 7;   // IN1 on module
const int Mixer_motorController_IN2   = 8;   // IN2 on module

// Trash conveyor (pinmap_mega2560.md).
const int TC_stepperMotorController_step   = 4;
const int TC_stepperMotorController_dir    = 22;
const int TC_stepperMotorController_enable = 23;
const int TC_servoMotor_pin                = 5;
const int TC_vacuumPumpRelay               = 24;
const int TC_stepperHomeLimit              = 34;
const int TC_leftFilmSensor                = 35;
const int TC_vacuumSensor                  = A0;

// ============================================================================
//  Constants
// ============================================================================

const int GATE_OPEN_DEG  = 55;
const int GATE_CLOSE_DEG = 0;

const unsigned long ENERGY_PRINT_INTERVAL_MS = 500;

// ── Trash conveyor tunables ────────────────────────────────────────────────
// Picker servo angles (deg).
const int TC_servoMinDeg  = 0;
const int TC_servoMaxDeg  = 180;
const int TC_servoUpDeg   = 0;
const int TC_servoDownDeg = 180;

// Vacuum pump relay (D24). Most relay boards energize on LOW; set false if
// your board is active-LOW, true if it energizes on HIGH.
const bool TC_pumpRelayActiveHigh = false;

// Vacuum pick detection (sensor on A0).
const float TC_analogReferenceV = 5.0;
const float TC_vacuumThresholdV = 2.5;
const bool  TC_vacuumDetectedWhenVoltageHigh = true;
const int   TC_servoPickStepDeg = 2;
const unsigned long TC_servoStepSettleMs     = 6;
const unsigned long TC_vacuumCheckIntervalMs = 1;
const unsigned long TC_servoReturnMsPerDeg   = 4;
const unsigned long TC_servoReturnMinMs      = 150;
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
const float TC_shredderPos   = -425.0;
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
  TC_BAG1_TO_SHREDDER,
  TC_SHREDDER_TO_BAG1,
  TC_MANUAL_MOVE,
  TC_DONE
};

TCState tcState = TC_WAIT_HOME;
int  tcLastLimitRead   = HIGH;
int  tcLimitState      = HIGH;
int  tcBag1TripsDone   = 0;
unsigned long tcLastLimitChange = 0;
int  tcCurrentServoAngle = TC_servoUpDeg;
bool tcPumpRunning     = false;

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
//  pinmap_mega2560.md. Vacuum PUMP control is intentionally omitted for now
//  (relay D24 unused); the vacuum SENSOR on A0 is still sampled so vacuum-based
//  pick detection works as soon as the pump is wired in and re-enabled.
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

bool tcBagStackEmptyConfirmed() {
  int emptySamples = 0;
  for (int i = 0; i < TC_bagSensorConfirmSamples; i++) {
    bool bagPresent = digitalRead(TC_leftFilmSensor) == TC_bagPresentState;
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
  TC_servoMotor.write(tcCurrentServoAngle);
}

void tcPumpOn() {
  digitalWrite(TC_vacuumPumpRelay, TC_pumpRelayActiveHigh ? HIGH : LOW);
  tcPumpRunning = true;
}

void tcPumpOff() {
  digitalWrite(TC_vacuumPumpRelay, TC_pumpRelayActiveHigh ? LOW : HIGH);
  tcPumpRunning = false;
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
  TC_stepper.stop();
  TC_stepper.disableOutputs();
  tcPumpOff();
  tcState = TC_DONE;
  Serial.println(message);
}

void tcPauseAtShredder() {
  // Release the bag over the shredder, then dwell.
  tcPumpOff();
  delay(TC_shredderPauseMs);
}

bool tcPickAtBag() {
  if (tcBagStackEmptyConfirmed()) {
    tcStopPickSequence(F("[TC] Bag stack empty - pick sequence stopped"));
    return false;
  }

  // Turn the vacuum pump on before the down stroke so suction builds while the
  // picker descends; tcVacuumDetectedDuringSettle() grabs as soon as the bag
  // seals against the cup.
  tcPumpOn();
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
    Serial.println(F("[TC] No vacuum detected at full down"));
  }

  tcReturnServoToUp();
  return true;
}

void tcStartHome() {
  tcBag1TripsDone = 0;
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
  if (tcState != TC_READY) {
    Serial.println(F("[TC] Home first (TC_HOME) and wait until at Bag 1"));
    return;
  }
  tcBag1TripsDone = 0;
  if (tcPickAtBag()) {
    tcMoveTo(TC_shredderPos, TC_BAG1_TO_SHREDDER);
    Serial.println(F("[TC] Pick cycle started"));
  }
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

    case TC_BAG1_TO_SHREDDER:
      tcPauseAtShredder();
      tcBag1TripsDone++;
      Serial.print(F("[TC] Bag 1 trip "));
      Serial.println(tcBag1TripsDone);
      tcMoveTo(TC_bag1Pos, TC_SHREDDER_TO_BAG1);
      break;

    case TC_SHREDDER_TO_BAG1:
      if (tcPickAtBag()) {
        tcMoveTo(TC_shredderPos, TC_BAG1_TO_SHREDDER);
      }
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
    case TC_BACK_TO_BAG1:      return F("HOMING");
    case TC_READY:             return F("READY");
    case TC_BAG1_TO_SHREDDER:
    case TC_SHREDDER_TO_BAG1:  return F("PICKING");
    case TC_MANUAL_MOVE:       return F("MANUAL");
    case TC_DONE:              return F("DONE");
    default:                   return F("WAIT_HOME");
  }
}

void tcPrintStatus() {
  Serial.print(F("[TC] state="));
  Serial.print(tcStateName());
  Serial.print(F(" pos_mm="));
  Serial.print(tcStepsToMm(TC_stepper.currentPosition()), 1);
  Serial.print(F(" limit="));
  Serial.print(tcLimitPressed() ? F("PRESSED") : F("OPEN"));
  Serial.print(F(" bag="));
  Serial.print(tcBagStackEmptyConfirmed() ? F("EMPTY") : F("DETECTED"));
  int vacuumRaw = analogRead(TC_vacuumSensor);
  Serial.print(F(" pump="));
  Serial.print(tcPumpRunning ? F("ON") : F("OFF"));
  Serial.print(F(" vac="));
  Serial.print(tcVacuumDetectedOnce() ? F("YES") : F("NO"));
  Serial.print(F(" vac_v="));
  Serial.print(tcRawToVoltage(vacuumRaw), 2);
  Serial.print(F(" trips="));
  Serial.print(tcBag1TripsDone);
  Serial.print(F(" servo="));
  Serial.println(tcCurrentServoAngle);
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
  Serial.print(F(" tc_bag="));
  Serial.print((digitalRead(TC_leftFilmSensor) == TC_bagPresentState) ? F("DETECTED") : F("EMPTY"));
  Serial.print(F(" tc_pump="));
  Serial.println(tcPumpRunning ? F("ON") : F("OFF"));
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

  } else if (cmd.startsWith("TC_SERVO ")) {
    int deg = cmd.substring(9).toInt();
    if (deg < TC_servoMinDeg || deg > TC_servoMaxDeg) {
      Serial.println(F("[TC] ERROR: servo angle must be 0-180"));
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

  // Trash conveyor - servo up, pump off, stepper disabled, awaiting TC_HOME
  pinMode(TC_stepperHomeLimit, INPUT_PULLUP);
  pinMode(TC_leftFilmSensor, INPUT);
  pinMode(TC_vacuumPumpRelay, OUTPUT);
  tcPumpOff();
  TC_servoMotor.attach(TC_servoMotor_pin);
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

/*
 * LunaRecycle - Arduino Uno Production Firmware
 * Copyright (C) re:3D, Inc. - All Rights Reserved
 *
 * Target board : Arduino Uno
 *
 * Subsystems:
 *   - Shredder Gate  : 2 RC servos (open / close)
 *   - DC Motor       : H-bridge speed + direction (continuous spin)
 *   - Energy Monitor : INA219 current / voltage / power sensor
 *
 * -- Serial protocol (9600 baud, newline-terminated) -------------------------
 *
 *   GATE_OPEN              Move both servos to open position (55 deg)
 *   GATE_CLOSE             Move both servos to closed position (0 deg)
 *   GATE_STATUS            Print current gate state
 *
 *   MOTOR_SET <spd> <dir>  Set DC motor.  spd = 0-255,  dir = FWD or REV
 *   MOTOR_STOP             Stop DC motor (PWM = 0)
 *   MOTOR_STATUS           Print current motor speed / direction
 *
 *   STATUS                 Print all subsystem states + one INA219 snapshot
 *   ESTOP                  Stop motor, close gate immediately
 *
 * INA219 readings are also streamed automatically every 500 ms:
 *   [ENERGY] pwm=150 dir=FWD V=12.34 I=1.234 P=15.23
 *
 * -- Pin map (Uno) ------------------------------------------------------------
 *
 *   Gate servo 1  ->  D9
 *   Gate servo 2  ->  D10
 *
 *   Motor ENB     ->  D3  (PWM, Timer 2)  [Channel B]
 *   Motor IN3     ->  D4
 *   Motor IN4     ->  D6
 *
 *   INA219        ->  A4 (SDA) / A5 (SCL)   [hardware I2C]
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// ============================================================================
//  Pin assignments
// ============================================================================

const int Mixer_shredderGateLeftServoMotor_pin = 9;
const int Mixer_shredderGateRightServoMotor_pin = 10;

const int Mixer_motorController_ENA   = 3;   // ENA on module (Timer 2 PWM)
const int Mixer_motorController_IN1   = 7;   // IN1 on module
const int Mixer_motorController_IN2   = 8;   // IN2 on module

// ============================================================================
//  Constants
// ============================================================================

const int GATE_OPEN_DEG  = 55;
const int GATE_CLOSE_DEG = 0;

const unsigned long ENERGY_PRINT_INTERVAL_MS = 500;

// ============================================================================
//  Objects
// ============================================================================

Servo Mixer_shredderGateLeftServoMotor;
Servo Mixer_shredderGateRightServoMotor;
Adafruit_INA219 Mixer_screwMotorCurrentSensor;

// ============================================================================
//  State
// ============================================================================

bool gateOpen  = false;
int  motorPwm  = 0;
bool motorFwd  = true;   // true = FWD, false = REV
bool inaOk     = false;

String cmdBuffer;
unsigned long lastEnergyPrint = 0;

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
//  Global STATUS
// ============================================================================

void printAllStatus() {
  Serial.print(F("[STATUS] gate="));
  Serial.print(gateOpen ? F("OPEN") : F("CLOSED"));
  Serial.print(F(" motor_pwm="));
  Serial.print(motorPwm);
  Serial.print(F(" motor_dir="));
  Serial.println(motorFwd ? F("FWD") : F("REV"));
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

  } else if (cmd == "STATUS") {
    printAllStatus();

  } else if (cmd == "ESTOP") {
    motorStop();
    gateCloseCmd();
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

  Serial.println(F("[SYSTEM] LunaRecycle Uno firmware ready"));
  Serial.println(F("[SYSTEM] Commands: GATE_OPEN, GATE_CLOSE, MOTOR_SET <spd> <FWD|REV>, MOTOR_STOP, STATUS, ESTOP"));
}

void loop() {
  readSerial();

  // Stream energy readings automatically every 500 ms
  unsigned long now = millis();
  if (now - lastEnergyPrint >= ENERGY_PRINT_INTERVAL_MS) {
    lastEnergyPrint = now;
    printEnergy();
  }
}

/*
 * Copyright (C) re:3D, Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 Mixer_screwMotorCurrentSensor;

// Motor driver pins
const int Mixer_motorController_PWM = 10;
const int Mixer_motorController_IN1 = 27;
const int Mixer_motorController_IN2 = 28;

const int Mixer_agitatorMotor_PWM = 44;
const int Mixer_agitatorMotor_IN3 = 42;
const int Mixer_agitatorMotor_IN4 = 43;

// Constant motor directions
const bool SCREW_FORWARD = false;
const bool AGITATOR_FORWARD = true;

// Print rate
const int PRINT_INTERVAL_MS = 500;

int screwSpeedPercent = 0;
int agitatorSpeedPercent = 0;
String serialCommand;
unsigned long lastPrintMs = 0;

int percentToPwm(int percent) {
  return map(constrain(percent, 0, 100), 0, 100, 0, 255);
}

void motorRun(int pwm, bool forward) {
  if (forward) {
    digitalWrite(Mixer_motorController_IN1, HIGH);
    digitalWrite(Mixer_motorController_IN2, LOW);
  } else {
    digitalWrite(Mixer_motorController_IN1, LOW);
    digitalWrite(Mixer_motorController_IN2, HIGH);
  }

  analogWrite(Mixer_motorController_PWM, constrain(pwm, 0, 255));
}

void agitatorMotorRun(int pwm, bool forward) {
  if (forward) {
    digitalWrite(Mixer_agitatorMotor_IN3, HIGH);
    digitalWrite(Mixer_agitatorMotor_IN4, LOW);
  } else {
    digitalWrite(Mixer_agitatorMotor_IN3, LOW);
    digitalWrite(Mixer_agitatorMotor_IN4, HIGH);
  }

  analogWrite(Mixer_agitatorMotor_PWM, constrain(pwm, 0, 255));
}

void setScrewSpeed(int percent) {
  screwSpeedPercent = constrain(percent, 0, 100);
  motorRun(percentToPwm(screwSpeedPercent), SCREW_FORWARD);
}

void setAgitatorSpeed(int percent) {
  agitatorSpeedPercent = constrain(percent, 0, 100);
  agitatorMotorRun(percentToPwm(agitatorSpeedPercent), AGITATOR_FORWARD);
}

void printCommandHelp() {
  Serial.println("Commands:");
  Serial.println("  SCREW <0-100>");
  Serial.println("  AGITATOR <0-100>");
  Serial.println("  BOTH <0-100>");
  Serial.println("  STOP");
}

bool parsePercentCommand(const String& cmd, const char* prefix, int& value) {
  String p = String(prefix);
  if (!cmd.startsWith(p + " ")) {
    return false;
  }

  String valueText = cmd.substring(p.length() + 1);
  valueText.trim();
  if (valueText.length() == 0) {
    return false;
  }
  for (unsigned int i = 0; i < valueText.length(); i++) {
    if (!isDigit(valueText.charAt(i))) {
      return false;
    }
  }
  value = valueText.toInt();
  return value >= 0 && value <= 100;
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  int percent = 0;
  if (parsePercentCommand(cmd, "SCREW", percent)) {
    setScrewSpeed(percent);
    Serial.print("Screw speed set to ");
    Serial.print(screwSpeedPercent);
    Serial.println("%");
  } else if (parsePercentCommand(cmd, "AGITATOR", percent)) {
    setAgitatorSpeed(percent);
    Serial.print("Agitator speed set to ");
    Serial.print(agitatorSpeedPercent);
    Serial.println("%");
  } else if (parsePercentCommand(cmd, "BOTH", percent)) {
    setScrewSpeed(percent);
    setAgitatorSpeed(percent);
    Serial.print("Both motor speeds set to ");
    Serial.print(percent);
    Serial.println("%");
  } else if (cmd == "STOP") {
    setScrewSpeed(0);
    setAgitatorSpeed(0);
    Serial.println("Both motors stopped");
  } else if (cmd == "HELP") {
    printCommandHelp();
  } else if (cmd.length() > 0) {
    Serial.println("Unknown command or percent out of range. Type HELP.");
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(serialCommand);
      serialCommand = "";
    } else {
      serialCommand += c;
    }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(Mixer_motorController_PWM, OUTPUT);
  pinMode(Mixer_motorController_IN1, OUTPUT);
  pinMode(Mixer_motorController_IN2, OUTPUT);

  pinMode(Mixer_agitatorMotor_PWM, OUTPUT);
  pinMode(Mixer_agitatorMotor_IN3, OUTPUT);
  pinMode(Mixer_agitatorMotor_IN4, OUTPUT);

  if (!Mixer_screwMotorCurrentSensor.begin()) {
    Serial.println("INA219 not found");
    while (1) {
      delay(50);
    }
  }

  delay(50);
  Serial.println("Starting serial-command motor test");
  printCommandHelp();

  setScrewSpeed(0);
  setAgitatorSpeed(0);
}

void loop() {
  readSerialCommands();

  unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintMs = now;

  float busVoltage_V = Mixer_screwMotorCurrentSensor.getBusVoltage_V();
  float shuntVoltage_mV = Mixer_screwMotorCurrentSensor.getShuntVoltage_mV();
  float current_A = Mixer_screwMotorCurrentSensor.getCurrent_mA() / 1000.0f;
  float power_W = Mixer_screwMotorCurrentSensor.getPower_mW() / 1000.0f;
  float loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000.0f);

  Serial.print("Screw:");
  Serial.print(screwSpeedPercent);
  Serial.print("% PWM:");
  Serial.print(percentToPwm(screwSpeedPercent));
  Serial.print(" Agitator:");
  Serial.print(agitatorSpeedPercent);
  Serial.print("% AgitatorPWM:");
  Serial.print(percentToPwm(agitatorSpeedPercent));
  Serial.print(" Load:");
  Serial.print(loadVoltage_V, 2);
  Serial.print("V");
  Serial.print(" I:");
  Serial.print(current_A, 3);
  Serial.print("A");
  Serial.print(" P:");
  Serial.print(power_W, 2);
  Serial.println("W");
}

/*
 * Copyright (C) re:3D, Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

/*
  FPU Upper Assembly Vacuum Sensor Test

  ZSE40-T1-62L analog output:
    1-5 V signal into Arduino Mega analog pin A0.

  Commands:
    on      Turn vacuum pump on
    off     Turn vacuum pump off
    status  Print sensor and motor status

  Pump safety:
    Never set TC_vacuumPumpRelay_IN1 LOW and TC_vacuumPumpRelay_IN2 HIGH. That would reverse and damage the pump.
*/

// Pins.
const int TC_vacuumSensor = A0;
const int TC_vacuumPumpRelay_ENA = 5;
const int TC_vacuumPumpRelay_IN1 = 6;
const int TC_vacuumPumpRelay_IN2 = 7;

// Sensor settings.
const float analogReferenceV = 5.0;
const float vacuumThresholdV = 2.5;
const bool vacuumDetectedWhenVoltageHigh = true;
const unsigned long printIntervalMs = 500;

// Pump settings.
const int pumpSpeed = 100;

unsigned long lastPrintMs = 0;
String serialCmd = "";
bool motorRunning = false;

void setup() {
  Serial.begin(9600);

  pinMode(TC_vacuumPumpRelay_ENA, OUTPUT);
  pinMode(TC_vacuumPumpRelay_IN1, OUTPUT);
  pinMode(TC_vacuumPumpRelay_IN2, OUTPUT);

  vacuumOff();

  Serial.println("Vacuum sensor test ready.");
  Serial.println("Commands: on, off, status");
}

void loop() {
  readSerial();
  printVacuumStatusIfDue();
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
  if (cmd == "on") {
    vacuumOn();
  } else if (cmd == "off") {
    vacuumOff();
    Serial.println("Motor off.");
  } else if (cmd == "status" || cmd == "s") {
    printVacuumStatus();
  } else if (cmd.length() > 0) {
    Serial.println("Use: on, off, status");
  }
}

void printVacuumStatusIfDue() {
  unsigned long now = millis();

  if (now - lastPrintMs < printIntervalMs) {
    return;
  }

  lastPrintMs = now;
  printVacuumStatus();
}

void printVacuumStatus() {
  int raw = analogRead(TC_vacuumSensor);
  float voltage = rawToVoltage(raw);
  bool detected = vacuumDetected(voltage);

  Serial.print("Vacuum: ");
  Serial.print(detected ? "DETECTED" : "not detected");
  Serial.print(" | raw: ");
  Serial.print(raw);
  Serial.print(" | voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V | motor: ");
  Serial.println(motorRunning ? "on" : "off");
}

float rawToVoltage(int raw) {
  return raw * (analogReferenceV / 1023.0);
}

bool vacuumDetected(float voltage) {
  if (vacuumDetectedWhenVoltageHigh) {
    return voltage >= vacuumThresholdV;
  }

  return voltage <= vacuumThresholdV;
}

void vacuumOn() {
  // Pump forward only. Do not reverse this pin order.
  digitalWrite(TC_vacuumPumpRelay_IN1, HIGH);
  digitalWrite(TC_vacuumPumpRelay_IN2, LOW);
  analogWrite(TC_vacuumPumpRelay_ENA, pumpSpeed);
  motorRunning = true;
  Serial.println("Motor on.");
}

void vacuumOff() {
  analogWrite(TC_vacuumPumpRelay_ENA, 0);
  digitalWrite(TC_vacuumPumpRelay_IN1, LOW);
  digitalWrite(TC_vacuumPumpRelay_IN2, LOW);
  motorRunning = false;
}

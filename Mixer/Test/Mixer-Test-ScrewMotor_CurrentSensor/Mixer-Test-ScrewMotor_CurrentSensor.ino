/*
 * Copyright (C) re:3D, Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 Mixer_screwMotorCurrentSensor;

// Motor driver pins
const int Mixer_motorController_PWM = 5;
const int Mixer_motorController_IN1 = 7;
const int Mixer_motorController_IN2 = 8;

// Constant motor settings
const int TEST_PWM = 140;   // 0-255
const bool FORWARD = false;

// Print rate
const int PRINT_INTERVAL_MS = 50;

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

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(Mixer_motorController_PWM, OUTPUT);
  pinMode(Mixer_motorController_IN1, OUTPUT);
  pinMode(Mixer_motorController_IN2, OUTPUT);

  if (!Mixer_screwMotorCurrentSensor.begin()) {
    Serial.println("INA219 not found");
    while (1) {
      delay(50);
    }
  }

  delay(50);
  Serial.println("Starting constant-speed motor test");

  motorRun(TEST_PWM, FORWARD);
}

void loop() {
  float busVoltage_V = Mixer_screwMotorCurrentSensor.getBusVoltage_V();
  float shuntVoltage_mV = Mixer_screwMotorCurrentSensor.getShuntVoltage_mV();
  float current_A = Mixer_screwMotorCurrentSensor.getCurrent_mA() / 1000.0f;
  float power_W = Mixer_screwMotorCurrentSensor.getPower_mW() / 1000.0f;
  float loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000.0f);

  Serial.print("PWM:");
  Serial.print(TEST_PWM);
  Serial.print(" Load:");
  Serial.print(loadVoltage_V, 2);
  Serial.print("V");
  Serial.print(" I:");
  Serial.print(current_A, 3);
  Serial.print("A");
  Serial.print(" P:");
  Serial.print(power_W, 2);
  Serial.println("W");

  delay(PRINT_INTERVAL_MS);
}

/*
 * Copyright (C) re:3D, Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

// Hall-effect sensor pin
const int Mixer_screwRotationSensor = 2;

// LED feedback
const int LED_PIN = 13;

// RPM calculation
volatile unsigned long pulseCount = 0;
unsigned long lastCalcMs = 0;
const int CALC_INTERVAL_MS = 500;   // recalculate RPM every 500 ms
const int PULSES_PER_REV   = 1;     // magnets per revolution on the screw shaft

void onPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);

  pinMode(Mixer_screwRotationSensor, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(Mixer_screwRotationSensor), onPulse, FALLING);

  Serial.println("Mixer screw rotation sensor test started.");
  Serial.println("Pin: Mixer_screwRotationSensor (D2)");
}

void loop() {
  unsigned long nowMs = millis();

  if (nowMs - lastCalcMs >= CALC_INTERVAL_MS) {
    // Snapshot and reset pulse count atomically
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float elapsedSec = (nowMs - lastCalcMs) / 1000.0f;
    float rpm = (pulses / (float)PULSES_PER_REV) / elapsedSec * 60.0f;
    lastCalcMs = nowMs;

    // LED mirrors magnet presence (LOW = magnet detected on US5881)
    bool magnetPresent = (digitalRead(Mixer_screwRotationSensor) == LOW);
    digitalWrite(LED_PIN, magnetPresent ? HIGH : LOW);

    Serial.print("Pulses:");
    Serial.print(pulses);
    Serial.print("  RPM:");
    Serial.print(rpm, 1);
    Serial.print("  Magnet:");
    Serial.println(magnetPresent ? "DETECTED" : "none");
  }
}

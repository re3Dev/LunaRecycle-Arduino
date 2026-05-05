#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

// Motor driver pins
const int PWM_PIN = 5;
const int IN1_PIN = 7;
const int IN2_PIN = 8;

// Constant motor settings
const int TEST_PWM = 140;   // 0-255
const bool FORWARD = false;

// Print rate
const int PRINT_INTERVAL_MS = 50;

void motorRun(int pwm, bool forward) {
  if (forward) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }

  analogWrite(PWM_PIN, constrain(pwm, 0, 255));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  if (!ina219.begin()) {
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
  float busVoltage_V = ina219.getBusVoltage_V();
  float shuntVoltage_mV = ina219.getShuntVoltage_mV();
  float current_A = ina219.getCurrent_mA() / 1000.0f;
  float power_W = ina219.getPower_mW() / 1000.0f;
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

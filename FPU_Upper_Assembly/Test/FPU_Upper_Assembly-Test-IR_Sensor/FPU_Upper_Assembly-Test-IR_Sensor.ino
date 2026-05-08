/*
  FPU Upper Assembly IR Sensor Test

  Wiring:
    IR sensor VCC -> 5V
    IR sensor GND -> GND
    IR sensor OUT -> pin 8

  Open Serial Monitor at 9600 baud.
*/

const int irSensorPin = 53;

// Most 3-pin IR obstacle sensors pull OUT LOW when triggered.
const bool triggeredState = LOW;

int lastSensorState = HIGH;

void setup() {
  Serial.begin(9600);
  pinMode(irSensorPin, INPUT);

  Serial.println("IR sensor test ready.");
  Serial.println("Move an object in front of the sensor.");
}

void loop() {
  int sensorState = digitalRead(irSensorPin);

  if (sensorState != lastSensorState) {
    lastSensorState = sensorState;

    if (sensorState == triggeredState) {
      Serial.println("IR sensor triggered.");
    } else {
      Serial.println("IR sensor clear.");
    }
  }

  delay(25);
}

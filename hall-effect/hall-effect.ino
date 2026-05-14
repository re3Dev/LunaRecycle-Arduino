const int HALL_PIN = 2;
const int LED_PIN = 13;  // Built-in LED on most Arduinos

void setup() {
  Serial.begin(115200);

  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Serial.println("US5881 Hall sensor test started.");
}

void loop() {
  int state = digitalRead(HALL_PIN);

  if (state == LOW) {
    // Sensor output LOW = magnet detected
    digitalWrite(LED_PIN, HIGH);
    Serial.println("MAGNET DETECTED - LED ON");
  } else {
    // Output HIGH = no magnet
    digitalWrite(LED_PIN, LOW);
    Serial.println("No magnet - LED OFF");
  }

  delay(100);
}

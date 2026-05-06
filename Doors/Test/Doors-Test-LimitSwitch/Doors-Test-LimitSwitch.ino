const int MIN_LIMIT_PIN = 2;
const int MAX_LIMIT_PIN = 3;

bool lastMin = false;
bool lastMax = false;

bool minHit() {
  return digitalRead(MIN_LIMIT_PIN) == LOW;  // pressed = LOW
}

bool maxHit() {
  return digitalRead(MAX_LIMIT_PIN) == LOW;  // pressed = LOW
}

void setup() {
  Serial.begin(9600);

  pinMode(MIN_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MAX_LIMIT_PIN, INPUT_PULLUP);

  Serial.println("Limit switch test ready");
  Serial.println("Press MIN or MAX switch.");
}

void loop() {
  bool currentMin = minHit();
  bool currentMax = maxHit();

  if (currentMin != lastMin) {
    lastMin = currentMin;

    if (currentMin) {
      Serial.println("MIN switch PRESSED");
    } else {
      Serial.println("MIN switch RELEASED");
    }
  }

  if (currentMax != lastMax) {
    lastMax = currentMax;

    if (currentMax) {
      Serial.println("MAX switch PRESSED");
    } else {
      Serial.println("MAX switch RELEASED");
    }
  }

  delay(20);
}

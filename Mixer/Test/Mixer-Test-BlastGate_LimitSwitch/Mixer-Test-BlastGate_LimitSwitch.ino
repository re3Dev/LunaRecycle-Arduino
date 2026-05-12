const int Mixer_blastGateLeftMinLimit = 2;
const int Mixer_blastGateLeftMaxLimit = 3;

bool lastMin = false;
bool lastMax = false;

bool minHit() {
  return digitalRead(Mixer_blastGateLeftMinLimit) == LOW;  // pressed = LOW
}

bool maxHit() {
  return digitalRead(Mixer_blastGateLeftMaxLimit) == LOW;  // pressed = LOW
}

void setup() {
  Serial.begin(9600);

  pinMode(Mixer_blastGateLeftMinLimit, INPUT_PULLUP);
  pinMode(Mixer_blastGateLeftMaxLimit, INPUT_PULLUP);

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

/*
Pin-46 NeoPixel Status Light Prove-Out
Drives the 7-pixel status light on Arduino Mega pin 46 only.
Use this to verify the production harness/data pin without any other firmware.
*/

#include <Adafruit_NeoPixel.h>

#define LED_PIN    46
#define LED_COUNT  7

Adafruit_NeoPixel statusLight(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastStepMs = 0;
int modeIndex = 0;

void showColor(uint8_t red, uint8_t green, uint8_t blue) {
  statusLight.fill(statusLight.Color(red, green, blue));
  statusLight.show();
}

void setup() {
  statusLight.begin();
  statusLight.setBrightness(48);
  statusLight.clear();
  statusLight.show();

  // Obvious power-on sequence.
  showColor(255, 0, 0);
  delay(300);
  showColor(0, 255, 0);
  delay(300);
  showColor(0, 0, 255);
  delay(300);
  showColor(255, 180, 0);
  delay(1000);

  lastStepMs = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastStepMs < 2000UL) {
    return;
  }
  lastStepMs = now;

  switch (modeIndex) {
    case 0:
      showColor(255, 180, 0);  // amber
      break;
    case 1:
      showColor(0, 255, 0);    // green
      break;
    case 2:
      showColor(0, 0, 255);    // blue
      break;
    case 3:
      showColor(255, 0, 0);    // red
      break;
    default:
      showColor(255, 255, 255); // white
      break;
  }

  modeIndex++;
  if (modeIndex > 4) {
    modeIndex = 0;
  }
}

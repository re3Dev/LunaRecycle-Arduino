/*Status Light Test Code
Use a 7-segment NeoPixel device to indicate machine status
LED flashing is non-blocking
For use on the LunaBotXS System; re:3D Inc, June 2026
*/

#include <Adafruit_NeoPixel.h>

#define LED_PIN    6       // Arduino pin connected to the NeoPixels
#define LED_COUNT  7       // Number of NeoPixels in your string

// Declare the NeoPixel object as statusLight
Adafruit_NeoPixel statusLight(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Tracking variables for non-blocking timing and state
unsigned long previousMillis = 0;
bool ledsAreOn = false;

void setup() {
  statusLight.begin();
  statusLight.show(); // Initialize all pixels to 'off'
}

void loop() {
  unsigned long currentMillis = millis();

  // Color assignments for the status light - color conventions from IEC 60204-1 (Safety of Machinery)
  uint32_t colorRed = statusLight.Color(255,0,0);       // Danger, Emergency, Fault
  uint32_t colorAmber = statusLight.Color(255,150,0);   // Warning, Off-Nominal
  uint32_t colorGreen = statusLight.Color(0,255,0);     // Normal, Safe
  uint32_t colorBlue = statusLight.Color(0,0,255);      // Operator Input Needed
  uint32_t colorWhite = statusLight.Color(255,255,255); // User-Defined

  // Set status color
  uint32_t statusColor = colorGreen;
  unsigned long interval = 500; // 500 = 1 Hz (500ms on, 500ms off) - set greater than 5000 for constant on

  // Set LEDs on
  if (interval > 5000) {
    statusLight.fill(statusColor);
    ledsAreOn = true;
    statusLight.show();
  // Check if it's time to toggle the LEDs (non-blocking)
  } else if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the last time the LEDs blinked

    if (!ledsAreOn) {
      // Turn all pixels ON
      statusLight.fill(statusColor);
      ledsAreOn = true;
    } else {
      // Turn all pixels OFF
      statusLight.clear();
      ledsAreOn = false;
    }

    statusLight.show(); // Push the updated colors to the hardware
  }

}

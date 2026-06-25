#include <Wire.h>
#include <Adafruit_AHTX0.h>

// The default I2C address for the TCA9548A I2C multiplexer
#define TCAADDR 0x70

// Create sensor instances
Adafruit_AHTX0 FPU_environmentalSensor;
Adafruit_AHTX0 GBX_environmentalSensor;
Adafruit_AHTX0 Dryer_regenExhaustEnvironmentalSensor;

// Helper function to select the desired TCA9548A port (0 to 7)
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial monitor to open

  Serial.println("TCA9548A & AHT20 Multi-Sensor Test");
  Wire.begin();

  // Initialize FPU Sensor on Port 0
  tcaselect(0);
  if (!FPU_environmentalSensor.begin()) {
    Serial.println("Could not find FPU_environmentalSensor on Port 0!");
    while (1) delay(10);
  }
  Serial.println("FPU_environmentalSensor initialized.");

  // Initialize GBX Sensor on Port 1
  tcaselect(1);
  if (!GBX_environmentalSensor.begin()) {
    Serial.println("Could not find GBX_environmentalSensor on Port 1!");
    while (1) delay(10);
  }
  Serial.println("GBX_environmentalSensor initialized.");

  // Initialize Dryer Regen Exhaust Sensor on Port 2
  tcaselect(2);
  if (!Dryer_regenExhaustEnvironmentalSensor.begin()) {
    Serial.println("Could not find Dryer_regenExhaustEnvironmentalSensor on Port 2!");
    while (1) delay(10);
  }
  Serial.println("Dryer_regenExhaustEnvironmentalSensor initialized.\n");
}

void loop() {
  sensors_event_t humidity, temp;

  // --- Read FPU Sensor ---
  tcaselect(0);
  FPU_environmentalSensor.getEvent(&humidity, &temp);
  Serial.print("FPU -> Temp: ");
  Serial.print(temp.temperature);
  Serial.print(" C | Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" % rH");

  // --- Read GBX Sensor ---
  tcaselect(1);
  GBX_environmentalSensor.getEvent(&humidity, &temp);
  Serial.print("GBX -> Temp: ");
  Serial.print(temp.temperature);
  Serial.print(" C | Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" % rH");

  // --- Read Dryer Regen Exhaust Sensor ---
  tcaselect(2);
  Dryer_regenExhaustEnvironmentalSensor.getEvent(&humidity, &temp);
  Serial.print("Dryer Regen -> Temp: ");
  Serial.print(temp.temperature);
  Serial.print(" C | Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" % rH");

  Serial.println("----------------------------------------------");
  delay(2000); // Wait 2 seconds before the next reading
}

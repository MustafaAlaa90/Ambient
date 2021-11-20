#include "CO2Sensor.h"

CO2Sensor co2Sensor(39, 0.99, 100);

void setup() {
  Serial.begin(115200);
  Serial.println("=== Initialized ===");
  co2Sensor.calibrate();
}

void loop() {
  float val = co2Sensor.read();
  Serial.print("CO2 value: ");
  Serial.println(val);

  delay(1000);
}

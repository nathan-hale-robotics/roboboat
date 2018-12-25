#include <Arduino.h>
#include <Wire.h>
#include <LSM303.h>

LSM303 compass;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = LSM303::vector<int16_t> {-280, -520, -780};
  compass.m_max = LSM303::vector<int16_t> {200, 0, -660};
}

int getCompass() {
  compass.read();
  return compass.heading();
}

int getGPSLat() {
  return 0;
}

int getGPSLon() {
  return 0;
}

void loop() {
  Serial.print("[CP");
  Serial.print(getCompass());
  Serial.print("]");

  Serial.print("[GP");
  Serial.print(getGPSLat());
  Serial.print(",");
  Serial.print(getGPSLon());
  Serial.println("]");
  delay(100);
}

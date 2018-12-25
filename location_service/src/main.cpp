#include <Arduino.h>
#include <Wire.h>
#include <LSM303.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

#define GPSECHO  true

LSM303 compass;

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
boolean usingInterrupt = false;
uint32_t timer = millis();

void useInterrupt(boolean);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = LSM303::vector<int16_t> {-280, -520, -780};
  compass.m_max = LSM303::vector<int16_t> {200, 0, -660};
  // setup GPS
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  useInterrupt(true);

  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  #ifdef UDR0
    if (GPSECHO) {
      if (c) {
        UDR0 = c;
      }
    }
  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

float getCompass() {
  compass.read();
  return compass.heading();
}

double getGPSLat() {
  return GPS.latitudeDegrees;
}

double getGPSLon() {
  return GPS.longitudeDegrees;
}

void loop() {
  if (! usingInterrupt) {
    char c = GPS.read();
    if (GPSECHO) {
      if (c) {
        // Serial.print(c);
      }
    }
  }

  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA()))
    return;
  }

  if (timer > millis()) {
    timer = millis();
  }

  if (millis() - timer > 2000) {
    timer = millis();
    if (GPS.fix) {
      Serial.print("[CP");
      Serial.print(getCompass());
      Serial.print("]");

      Serial.print("[GP");
      Serial.print(getGPSLat(), 8);
      Serial.print(",");
      Serial.print(getGPSLon(), 8);
      Serial.println("]");
      delay(100);
    }
  }

}

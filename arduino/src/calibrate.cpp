
#include <Wire.h>
#include <LSM303.h>

LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
char report[80];

void calibrate() {
  LSM303 compass;
  for (int i = 0; i < 200; i++) {
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);

    snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}, i: %d",
      running_min.x, running_min.y, running_min.z,
      running_max.x, running_max.y, running_max.z, i);
    Serial.println(report);

    delay(100);
  }
}

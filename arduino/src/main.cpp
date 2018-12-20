#include <Arduino.h>
#include <Servo.h>

Servo myservo;

void setup() {
  myservo.attach(2);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}

char waitForChar() {
  delay(1);
  while (Serial.available() <= 0);
  return Serial.read();
}

void serialFlush() {
  delay(1);
  while(Serial.available() > 0) {
    delay(1);
    Serial.read();
  }
}

char * getUntil(char term) {
  uint8_t length = 20;
  char *msg = (char *) malloc(sizeof(char) * (length + 1));
  char b;
  uint8_t i = 0;
  while (true) {
    b = waitForChar();
    Serial.print("I received char: num: ");
    Serial.print(b, DEC);
    Serial.print(", let: ");
    Serial.println(b);
    if (b == term || i > length) { // end of msg, or over length
      serialFlush();
      if (i > length) {
        msg[i] = b;
      }
      break;
    }
    msg[i] = b;
    i++;
  }
  msg[i + 1] = '\0';
  delay(1);
  return msg;
}

void loop() {
  char *msg = getUntil('\n');

  Serial.print("I received: ");
  Serial.println(msg);

  free(msg);
}

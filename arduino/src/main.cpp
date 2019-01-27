#include <Arduino.h>
#include <Servo.h>
#include <gps.cpp>

Servo rightMotor;
Servo leftMotor;

void setup() {
  rightMotor.attach(2);
  leftMotor.attach(3);
  Serial.begin(115200);
  setupGPS();
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

void getCommand(char *buff) {
  char b;
  uint8_t i = 0;
  bool inCommand = false;
  while (true) {
    b = waitForChar();
    Serial.print("I received char: num: ");
    Serial.print(b, DEC);
    Serial.print(", let: ");
    Serial.println(b);
    if (b == '[') {
      inCommand = true;
      i = 0;
      continue;
    }
    if ((b == ']' && inCommand) || i >= 20) {
      serialFlush();
      inCommand = false;
      buff[i] = '\0';
      return;
    }
    buff[i] = b;
    i++;
  }
}

void runCommand(char *command, char *arg) {
  if (strcmp(command, "MR") == 0) {
    int val = map(atoi(arg), 0, 100, 0, 180);
    rightMotor.write(val);
  } else if (strcmp(command, "ML") == 0) {
    int val = map(atoi(arg), 0, 100, 0, 180);
    leftMotor.write(val);
  } else if (strcmp(command, "GP") == 0) {
    sendGPS();
  }
}

void loop() {
  char msg[20 + 1];
  getCommand(msg);

  Serial.println("msg: ");
  Serial.println(msg);
  Serial.println("Length: ");
  Serial.println(strlen(msg));

  if (strlen(msg) <= 2) {
    return;
  }

  char command[2 + 1];
  for (uint8_t i = 0; i < 2; i++) {
    command[i] = msg[i];
    if (msg[i] == '\0') {
      break;
    }
  }
  command[2] = '\0';

  Serial.println("Command: ");
  Serial.println(command);

  char arg[18 + 1];
  for (uint8_t i = 0; i < 18; i++) {
    arg[i] = msg[i + 2];
    if (msg[i + 2] == '\0') {
      break;
    }
  }
  arg[18] = '\0';

  Serial.println("Arg: ");
  Serial.println(arg);

  runCommand(command, arg);
}

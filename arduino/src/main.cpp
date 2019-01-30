//190120combinedStringMessHardwareSerial
#include <Arduino.h>
#include <Wire.h>
#include <LSM303.h>
#include <Adafruit_GPS.h>
#include <Servo.h>
//for receiver
//#define THROTTLE_SIGNAL_IN 0 // INTERRUPT 0 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
//#define THROTTLE_SIGNAL_IN_PIN 2 // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead
#define NEUTRAL_THROTTLE 1500 // this is the duration in microseconds of neutral throttle on an electric RC Car

volatile int RightMotorIn = NEUTRAL_THROTTLE; // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartRightMotor = 0; // set in the interrupt
volatile boolean bNewRightMotorSignal = false;

#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

#define RIGHT_MOTOR_IN 2
#define LEFT_MOTOR_IN 3
#define AUX_IN_PIN 19 //manual (1), disabled (2), auto (3)

// Assign your channel out pins
#define RIGHTMOTOR_OUT 5
#define LEFTMOTOR_OUT 6

#define RIGHT_FLAG 1
#define LEFT_FLAG 2
#define AUX_FLAG 4
volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unRightMotor = 1500;
volatile uint16_t unLeftMotor = 1500;
volatile uint16_t unAuxIn = 2000;
uint32_t ulRightMotorStart;
uint32_t ulLeftMotorStart;
uint32_t ulAuxStart;

Servo rightMotor;
Servo leftMotor;
LSM303 compass;

////////////// move to setup loop dad
String msg = "";
bool msgComplete = false;


uint32_t timer = millis();

void calcRightInput();
void calcLeftInput();
void calcAux();

void setup() {
  msg.reserve(21);

  rightMotor.attach(RIGHTMOTOR_OUT); // was 2
  leftMotor.attach(LEFTMOTOR_OUT); // was 3
  //set to initial value
  rightMotor.writeMicroseconds(1500);
  leftMotor.writeMicroseconds(1500);

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
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);

  pinMode(RIGHT_MOTOR_IN, INPUT_PULLUP);
  pinMode(LEFT_MOTOR_IN, INPUT_PULLUP);
  pinMode(AUX_IN_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RIGHT_MOTOR_IN),calcRightInput,CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_MOTOR_IN),calcLeftInput,CHANGE);
  attachInterrupt(digitalPinToInterrupt(AUX_IN_PIN),calcAux,CHANGE);
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  if (GPSECHO) {
    if (c) {
      Serial.print(c);
    }
  }
}

float getCompass() {
  Serial.println("entered getCompass");
  compass.read();
  Serial.println("starting compass.heading");
  return compass.heading();
  Serial.println("exit getCompass");
}

double getGPSLat() {
  return GPS.latitudeDegrees;
}

double getGPSLon() {
  return GPS.longitudeDegrees;
}

void runCommand(String command) {
  int num = 0;
  String str = "";
  if(command.startsWith("MR")) {
    Serial.println("right motor initiated");
    str = command.substring(2);
    num = str.toInt();
    num = constrain(num, 0, 100);
    num = map(num, 0, 100, 1000, 2000);
    rightMotor.writeMicroseconds(num);
    Serial.println("MR set to: " + String(num));
  } else if (command.startsWith("ML")) {
    Serial.println("left motor initiated");
    str = command.substring(2);
    num = str.toInt();
    num = constrain(num, 0, 100);
    num = map(num, 0, 100, 1000, 2000);
    leftMotor.writeMicroseconds(num);
    Serial.println("ML set to: " + String(num));
  } else if (command.startsWith("CP")) {
    Serial.println("CP is: " + String(getCompass()));
  } else if( command.startsWith("GP")) {
    Serial.println("GP is: " + String(getGPSLat()) + ", -" + String(getGPSLon()));
  }
}

void serialEvent(){
  while(Serial.available()) {
    char c = (char)Serial.read();
    if(c == '[') {
      msg = "";
      msgComplete = false;
    } else if(c == ']') {
      msgComplete = true;
    } else {
      msg += c;
    }
  }
}

void loop() {
  if(bNewRightMotorSignal) {
    bNewRightMotorSignal = false;
  }

  if (unAuxIn <= 1300) { // manual mode
    rightMotor.writeMicroseconds(unRightMotor);
    leftMotor.writeMicroseconds(unLeftMotor);

  } else if (unAuxIn > 1300 && unAuxIn < 1700) { // disabled
    rightMotor.writeMicroseconds(1500);
    leftMotor.writeMicroseconds(1500);
  } else if (unAuxIn >= 1700) { // auto mode
    if (msgComplete) {
      Serial.println("msg: ");
      Serial.println(msg);

      runCommand(msg);
      msg = "";
      msgComplete = false;
    }
  }
  //////////////////////////
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    } else {
      Serial.println("no fix");
    }
  }

  delay(100);
}

void calcAux() {
  if(digitalRead(AUX_IN_PIN) == HIGH) {
    ulAuxStart = micros();
  } else {
    unAuxIn = (uint16_t)(micros() - ulAuxStart);
  }
}

void calcRightInput() {
  if(digitalRead(RIGHT_MOTOR_IN) == HIGH) {
    ulRightMotorStart = micros();
  } else {
    unRightMotor = (int)(micros() - ulRightMotorStart);
  }
}

void calcLeftInput() {
  if (digitalRead(LEFT_MOTOR_IN) == HIGH) {
    ulLeftMotorStart = micros();
  } else {
    unLeftMotor = (int)(micros() - ulLeftMotorStart);
  }
}

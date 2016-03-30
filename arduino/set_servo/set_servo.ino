#include <Servo.h>

#define SERIAL_BAUD 115200

#define BASE_PIN 9 // base motor
#define EL_PIN 10 // elbow motor
#define LIFT_PIN 11 // lift motor
#define STATUS_LED_PIN 13 // status led

#define BUFFER_SIZE 2 // incoming buffer

Servo arm[3];

byte incomingBuffer[BUFFER_SIZE];

void setup() {
  // begin serial port
  Serial.begin(SERIAL_BAUD);

  // attach servos
  arm[0].attach(BASE_PIN);
  arm[1].attach(EL_PIN);
  arm[2].attach(LIFT_PIN);

  arm[0].write(93);
  arm[1].write(80);
  arm[2].write(80);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    Serial.readBytes(incomingBuffer, BUFFER_SIZE);
    digitalWrite(STATUS_LED_PIN, HIGH);
    arm[incomingBuffer[0]].write(incomingBuffer[1]);
  }
  digitalWrite(STATUS_LED_PIN, LOW);
}

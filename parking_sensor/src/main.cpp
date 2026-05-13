#include <Arduino.h>

#define SIGNAL_PIN 2

void setup() {
  Serial.begin(9600);
  pinMode(SIGNAL_PIN, INPUT);
  Serial.println("us | cm");
}

void loop() {
  unsigned long us = pulseIn(SIGNAL_PIN, HIGH, 500000UL);

  if (us == 0) {
    Serial.println("timeout");
    delay(200);
    return;
  }

  uint16_t cm = us / 58;

  Serial.print(us);
  Serial.print(" | ");
  Serial.println(cm);

  delay(200);
}

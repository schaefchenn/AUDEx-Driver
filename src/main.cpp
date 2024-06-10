#include <Arduino.h>
#include <driver.h>

driver AUDEx;

void setup() {
  Serial.begin(115200); // Attach the servo after it has been detached
  AUDEx.setup();
}


void loop() {
  AUDEx.XBOXdriving();
}
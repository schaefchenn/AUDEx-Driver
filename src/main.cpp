#include <Arduino.h> // Include the Arduino library
#include <driver.h> // Include the driver header file

// Create an instance of the driver class
driver AUDEx;

void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);

  // Call the setup function of the driver instance
  AUDEx.setup();
}

void loop() {
  // Continuously call the XBOXdriving function of the driver instance
  AUDEx.XBOXdriving();
  // dont be annoying git
}

#include <Arduino.h>
#include "driver.h"

// Create an instance of the driver class
driver AUDEx;

void setup() {
    // Initialize serial communication at 115200 baud rate
    Serial.begin(115200);

    // Call the setup function of the driver instance
    AUDEx.setup();
    AUDEx.driveMode = 1; // 1 is for driving with the XBOX Series X Controller
}

void loop() {
    // Continuously call the driving function of the driver instance
    AUDEx.driving(AUDEx.driveMode);
}

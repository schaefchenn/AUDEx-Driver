#include "driver.h" // Include the driver header file
#include <Arduino.h> // Include the Arduino library

// Constructor for the driver class
driver::driver() {

}

// Setup function to initialize the driver
void driver::setup() {

    // Initialize serial communication
    Serial.println("\nInitializing Driver...");
    Serial.println("Starting NimBLE Client");
    
    // Setting up XBOX controller connection
    xboxController.begin();

    // Set up the built-in LED
    pinMode(ledPin, OUTPUT);

    // Steering Servo setup
    absimaServo.attach(steeringPin);
    Serial.println("Steering Ready");

    // Motor setup - the Absima motor controller allows the motor to be treated as a servo
    absimaMotor.attach(motorPin);
    absimaMotor.writeMicroseconds(1500); // Neutral position for the motor
    Serial.println("Motor Ready");

    // Wait 10 seconds to allow for setup completion
    delay(10000);
    Serial.println("Setup done");
}

// Function to demonstrate vibration on the XBOX controller
void driver::demoVibration() {
    XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
    repo.v.select.center = true;   // Enable center vibration
    repo.v.select.left = false;    // Disable left vibration
    repo.v.select.right = false;   // Disable right vibration
    repo.v.select.shake = false;   // Disable shake
    repo.v.power.center = 80;      // Set center vibration power to 80%
    repo.v.timeActive = 35;        // Set active time to 0.35 seconds
    Serial.println("\nController Connected Successfully");
    xboxController.writeHIDReport(repo); // Send vibration report to controller
}

// Function to handle XBOX driving inputs
void driver::XBOXdriving() {
    xboxController.onLoop(); // Process controller loop
    if (xboxController.isConnected()) { // Check if controller is connected
        if (xboxController.isWaitingForFirstNotification()) {
            Serial.println("waiting for first notification");
        } else {
            if (flag == 0) {
                demoVibration(); // Demonstrate vibration on first connection
                digitalWrite(LED_BUILTIN, HIGH); // Turn on built-in LED
                Serial.println("Address: " + xboxController.buildDeviceAddressStr()); // Print controller address
                Serial.println("battery " + String(xboxController.battery) + "%"); // Print battery status
                flag += 1;
            }
            
            // Read and map joystick horizontal value to steering angle
            joyLHoriValue = (float)xboxController.xboxNotif.joyLHori;
            steeringAngle = map(joyLHoriValue, 0, 65535, 0 + steeringOffset, 180 - steeringOffset);
            
            // Center steering angle if within tolerance
            if (abs(steeringAngle - centerSteeringAngle) <= centerSteeringTolerance) {
                steeringAngle = centerSteeringAngle;
            }
            absimaServo.write(steeringAngle); // Set servo to steering angle

            // Read and map trigger values to throttle value
            float gas = (float)xboxController.xboxNotif.trigRT;
            float brake = (float)xboxController.xboxNotif.trigLT;
            throttleValue = map(gas - brake, -1023, 1023, 1000, 2000); // Map throttle value
            absimaMotor.writeMicroseconds(throttleValue); // Set motor throttle
            
            // Print throttle and steering values
            Serial.print("throttle value: ");
            Serial.print(throttleValue);
            Serial.print("\t\tsteering angle: ");
            Serial.println(steeringAngle);
        }
    } else {
        // Restart ESP if connection failed multiple times
        if (xboxController.getCountFailedConnection() > 2) {
            ESP.restart();
        }

        // Handle disconnection
        if (flag >= 1){
            digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED
            Serial.println("not connected");
            flag = 0;
        }
    }
}

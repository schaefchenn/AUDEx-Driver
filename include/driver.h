#ifndef DRIVER_H
#define DRIVER_H

// Include necessary libraries
#include <XboxSeriesXControllerESP32_asukiaaa.hpp> // Xbox controller library for ESP32
#include <ESP32Servo.h> // Servo library for ESP32

// Define the driver class
class driver {
  public:
    driver();  // Constructor

    void setup(); // Setup function to initialize the driver
    void XBOXdriving(); // Function to handle XBOX controller driving
    void parkassistant(); // Function for park assistant feature

  private: // Private members - constants and variables
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController; // Xbox controller object

    Servo absimaServo; // Servo object for steering
    Servo absimaMotor; // Servo object for motor control

    const int ledPin = 2; // Pin for built-in LED
    const int steeringPin = 25; // Pin for steering servo
    const int motorPin = 26; // Pin for motor servo
    const float steeringOffset = 30; // Steering offset to prevent servo damage
    const float centerSteeringAngle = 90; // Center angle for steering (to account for joystick drift)
    const float centerSteeringTolerance = 3; // Tolerance for centering the steering (to account for joystick drift)
    
    int flag = 0; // Flag variable to track state

    float joyLHoriValue = 65535/2; // Initial value for joystick horizontal position
    float steeringAngle = 90; // Initial steering angle
    int throttleValue = 0; // Initial throttle value

    void demoVibration(); // Private function to demonstrate controller vibration
};

#endif

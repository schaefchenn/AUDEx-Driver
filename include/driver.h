#ifndef DRIVER_H
#define DRIVER_H

// Include necessary libraries
#include <XboxSeriesXControllerESP32_asukiaaa.hpp> // Xbox controller library for ESP32
#include <ESP32Servo.h> // Servo library for ESP32

struct XBOX {
    bool isConnected;
    float steeringAngle;
    float throttleValue;
    bool buttonA;
    bool buttonB;
    bool buttonX;
    bool buttonY;
    bool buttonLB;
    bool buttonRB;
    bool buttonStart;
    bool buttonSelect;
    bool buttonLStick;
    bool buttonRStick;
    bool buttonCrossUP;
    bool buttonCrossDOWN;
    bool buttonCrossLEFT;
    bool buttonCrossRIGHT;
};


struct CANBUS {
    bool extended;
    bool rtr;
    uint32_t id;
    uint8_t length;
    uint8_t data[8];  // Assuming maximum data length is 8 bytes
    uint8_t driveMode;
    uint8_t throttleValue;
    uint8_t steeringAngle;
    uint8_t acknowledged;
};

struct CanMessage {
  uint8_t driverReady;
};

// Define the driver class
class driver {
  public:
    driver();  // Constructor

    void setup(); // Setup function to initialize the driver
    XBOX getXboxData(); // Function to handle XBOX controller driving
    CANBUS getCanData(); // Function for park assistant feature
    void sendCanData(int driverReady);
    int driving(int driveMode, int CANthrottleValue, int CANsteerignAngle);
    


  private: // Private members - constants and variables
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController; // Xbox controller object

    Servo absimaServo; // Servo object for steering
    Servo absimaMotor; // Servo object for motor control

    const int TX_GPIO_NUM = 17;  // Connects to CTX
    const int RX_GPIO_NUM = 16;  // Connects to CRX

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
    int throttleLimit = 1600;
    int debounceDelay = 100;

    void demoVibration(); // Private function to demonstrate controller vibration

  public:
    int driveMode;
    int CANthrottleValue;
    int CANsteerignAngle;

    int driverReady = 0;
};

#endif

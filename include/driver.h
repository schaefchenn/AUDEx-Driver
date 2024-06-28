#ifndef DRIVER_H
#define DRIVER_H

// Include necessary libraries
#include <XboxSeriesXControllerESP32_asukiaaa.hpp> // Xbox controller library for ESP32
#include <ESP32Servo.h> // Servo library for ESP32
#include "SBUS.h"

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

struct Driver {
    int driveMode;
    float throttleValue;
    float steeringAngle;
    int CANstatus;
    int CANflag;
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
  uint8_t throttleValue;
};

enum rx_mode_enum{
  PPM_MODE = 0,
  SBUS_MODE
};

#define RX_THROTTLE_CH  0
#define RX_STEERING_CH  1
#define RX_MAX_CHANNELS 8

struct PPMData {
  bool available = 0;
  bool failsafe = 0;
  uint16_t channels[RX_MAX_CHANNELS] = {1500,1500,1500,1500,1500,1500,1500,1500};
};
volatile static PPMData _ppm_data;  // ppm data buffer used in interrupt.

struct SBUSData{
  float channelsCal[RX_MAX_CHANNELS] = {1, 1, 1, 1, 1, 1, 1, 1};
  uint16_t channels[RX_MAX_CHANNELS] = {1500,1500,1500,1500,1500,1500,1500,1500};
  bool failSafe;
  bool lostFrame;
  bool available;
};

enum drive_mode_enum{
  DRIVE_MODE_XBOX = 1,
  DRIVE_MODE_CAN,
  DRIVE_MODE_XBOX_LIMITED,
  DRIVE_MODE_RADIO_RX
};

// Define the driver class
class driver {
  public:
    driver();  // Constructor

    void setup(); // Setup function to initialize the driver
    XBOX getXboxData(); // Function to handle XBOX controller driving
    CANBUS getCanData(); // Function for park assistant feature
    void sendCanData(int driverReady, float throttleValue);
    bool getPPMData(PPMData& data); // get throttle and steering data using ppm receiver
    bool getSbusData(SBUSData& data);
    Driver driving(int driveMode, int CANthrottleValue, int CANsteerignAngle, int CANstatus, int CANflag);
    

  private: // Private members - constants and variables
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController; // Xbox controller object

    Servo absimaServo; // Servo object for steering
    Servo absimaMotor; // Servo object for motor control

    static void PPM_ISR(); // isr for ppm receiver signal
    SBUS sbusReceiver = SBUS(Serial1);  // hardware serial 1 for sbus receiver
    const int receiverPin = 4;  // radio receiver pin
    const uint8_t rxMode = PPM_MODE;  // default rx mode

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
    int throttleLimit = 0;
    int gear = 0;
    int debounceDelay = 100;

    void demoVibration(); // Private function to demonstrate controller vibration

  public:
    int driveMode = 0;
    int CANstatus = -1;
    int CANflag = 1;
    int CANthrottleValue;
    int CANsteerignAngle;
    int throttleValue = 0; // Initial throttle value
    int driverReady = 0;
};

#endif

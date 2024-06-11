#ifndef DRIVER_H
#define DRIVER_H

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <ESP32Servo.h>

class driver {
  public:
    driver();  // Constructor

    void setup();
    void XBOXdriving();
    void parkassistant();

  private: // hier alle konstanten Variablen 
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

    Servo absimaServo;
    Servo absimaMotor;

    const int ledPin = 2; // Buildin LED
    const int steeringPin = 25;
    const int motorPin = 26;
    const float steeringOffset = 30; // to not damage the servo
    const float centerSteeringAngle = 90; // for XBOX Joystick drift
    const float centerSteeringTolerance = 3; // for XBOX Joystick drift
    
    int flag = 0;

    float joyLHoriValue = 65535/2;
    float steeringAngle = 90;
    int throttleValue = 0;

    void demoVibration();
};

#endif

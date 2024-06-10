#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <ESP32Servo.h>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

// create servo object
Servo absimaServo;
Servo absimaMotor;

#define motorPin 26
#define steeringPin 25
#define ledPin 2

const float maxSteeringAngle = 170.0;
const float minSteeringAngle = 30.0;
const float centerSteeringAngle = 90.0;
const float centerTolerance = 3.0;

int flag = 0;
float throttleValue = 0;

void setup() {
  Serial.begin(115200); // Attach the servo after it has been detached

  Serial.begin(115200);
  Serial.println("Starting NimBLE Client");
  xboxController.begin();
  pinMode(ledPin, OUTPUT);

  absimaServo.attach(steeringPin);
  Serial.println("Steering Ready");

  absimaMotor.attach(motorPin); // Attach the servo after it has been detached
  absimaMotor.writeMicroseconds(1500);
  delay(10000); // wait 10s
  Serial.println("Setup done");

}

void demoVibration() {
    XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
    repo.v.select.center = true;
    repo.v.select.left = false;
    repo.v.select.right = false;
    repo.v.select.shake = false;
    repo.v.power.center = 80;  // 80% power
    repo.v.timeActive = 35;    // 0.35 second
    Serial.println("\nController Connected Successfully");
    xboxController.writeHIDReport(repo);
}


void loop() {
  //absimaMotor.write(90);
  //absimaMotor.writeMicroseconds(1600);
  xboxController.onLoop();
  if (xboxController.isConnected()) {
    if (xboxController.isWaitingForFirstNotification()) {
      Serial.println("waiting for first notification");
    } else {
        if (flag == 0) {
          demoVibration();
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.println("Address: " + xboxController.buildDeviceAddressStr());
          Serial.println("battery " + String(xboxController.battery) + "%");
          flag += 1;
        }
          
        float joyLHoriValue = (float)xboxController.xboxNotif.joyLHori;
        float steeringAngle = map(joyLHoriValue,0,65535,30,170);
        absimaServo.write(steeringAngle);

        float gas = (float)xboxController.xboxNotif.trigRT;
        float brake = (float)xboxController.xboxNotif.trigLT;
        throttleValue = map(gas - brake,-1023,1023,1000,2000);
        absimaMotor.write(throttleValue);
          
        Serial.print("throttle value: ");
        Serial.print(throttleValue);
        Serial.print("\t\tsteering angle: ");
        Serial.println(steeringAngle);
        }
  } else {
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }

    if (flag >= 1){
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("not connected");
      flag = 0;
    }
  }
}
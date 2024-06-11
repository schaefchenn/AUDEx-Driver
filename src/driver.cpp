#include "driver.h"
#include <Arduino.h>

driver::driver() {

}

void driver::setup(){
    Serial.println("\nInitializing Driver...");
    Serial.println("Starting NimBLE Client");
    xboxController.begin();

    pinMode(ledPin, OUTPUT);

    // steering Servo setup
    absimaServo.attach(steeringPin);
    Serial.println("Steering Ready");

    absimaMotor.attach(motorPin); // Attach the servo after it has been detached
    absimaMotor.writeMicroseconds(1500);
    Serial.println("Motor Ready");

    delay(10000); // wait 10s
    Serial.println("Setup done");
}

void driver::demoVibration() {
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

void driver::XBOXdriving() {
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
            
            joyLHoriValue = (float)xboxController.xboxNotif.joyLHori;
            steeringAngle = map(joyLHoriValue,0 , 65535, 0 + steeringOffset, 180 - steeringOffset);
            if (abs(steeringAngle - centerSteeringAngle) <= centerSteeringTolerance) {
                steeringAngle = centerSteeringAngle;
            }
            absimaServo.write(steeringAngle);

            float gas = (float)xboxController.xboxNotif.trigRT;
            float brake = (float)xboxController.xboxNotif.trigLT;
            throttleValue = map(gas - brake,-1023,1023,1000,2000);
            absimaMotor.writeMicroseconds(throttleValue);
          
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

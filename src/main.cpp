#include <Arduino.h>
#include <ESP32Servo.h>

#define motorPin 26
#define steeringPin 18

// create servo object
Servo absimaServo;
Servo absimaMotor;

void setup() {
  Serial.begin(115200);
  absimaServo.attach(steeringPin); // Attach the servo after it has been detached
  //absimaMotor.setPeriodHertz(90);      // Standard 50hz servo
  absimaMotor.attach(motorPin); // Attach the servo after it has been detached
}

void loop() {
  absimaMotor.write(90);
  // Move servo from 0 to 180 degrees

  for (int i = 0; i <= 180; i += 1) {
    absimaServo.write(i);
    delay(10); // Adjust delay for desired speed

    Serial.print("degrees: ");
    Serial.print(i);
    Serial.println(" °");
  }

  delay(1000); // Add a delay between cycles (optional)

  // Move servo back from 180 to 0 degrees
  for (int i = 180; i >= 0; i -= 1) {
    absimaServo.write(i);
    delay(10); // Adjust delay for desired speed

    Serial.print("degrees: ");
    Serial.print(i);
    Serial.println(" °");
  }

  delay(1000); // Add a delay between cycles (optional)
}

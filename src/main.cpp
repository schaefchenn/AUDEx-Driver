#include <Arduino.h>
#include <ESP32Servo.h>


#define motorPin 26
#define steeringPin 25

// create servo object
Servo absimaServo;
Servo absimaMotor;

void setup() {
  Serial.begin(115200); // Attach the servo after it has been detached
  absimaServo.attach(steeringPin);

  absimaMotor.attach(motorPin); // Attach the servo after it has been detached
  absimaMotor.writeMicroseconds(1500);
  delay(10000); // wait 10s
  Serial.println("Setup done");

}

void loop() {
  //absimaMotor.write(90);
  absimaMotor.writeMicroseconds(1600);
  
  
  // Move servo from 0 to 180 degrees
  for (int i = 30; i <= 150; i += 1) {
    absimaServo.write(i);
    delay(10); // Adjust delay for desired speed

    Serial.print("degrees: ");
    Serial.print(i);
    Serial.println(" °");
  }

  delay(1000); // Add a delay between cycles (optional)

  // Move servo back from 180 to 0 degrees
  for (int i = 150; i >= 30; i -= 1) {
    absimaServo.write(i);
    delay(10); // Adjust delay for desired speed

    Serial.print("degrees: ");
    Serial.print(i);
    Serial.println(" °");
  }

  delay(1000); // Add a delay between cycles (optional)
}

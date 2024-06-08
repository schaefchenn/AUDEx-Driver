#include <Arduino.h>
#include <ESP32Servo.h>

// create servo object
Servo absimaServo;
int absimaServoPin = 15;

void setup() {
  Serial.begin(115200);
  absimaServo.attach(absimaServoPin); // Attach the servo after it has been detached
}

void loop() {
  // Move servo from 0 to 180 degrees
  for (int i = 0; i <= 180; i += 1) {
    absimaServo.write(i);
    delay(10); // Adjust delay for desired speed

    Serial.print("degrees: ");
    Serial.print(i);
    Serial.println(" °");
  }

  // Move servo back from 180 to 0 degrees
  for (int i = 180; i >= 0; i -= 1) {
    absimaServo.write(i);
    delay(10); // Adjust delay for desired speed

    Serial.print("degrees: ");
    Serial.print(i);
    Serial.println(" °");
  }

  delay(1000); // Add a delay between cycles (optional)
  // ich mache hier änderungen lol
  int i;
  // ich ändere mehr stuff lol
  
}

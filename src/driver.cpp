#include "driver.h" // Include the driver header file
#include <Arduino.h> // Include the Arduino library
#include <CAN.h>
#include <display.h>

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


    // Set CAN the pins
    CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

    // start the CAN bus at 1Mbps
    if (!CAN.begin (1E6)) {
        Serial.println ("Starting CAN failed!");
        while (1);
    } else {
        Serial.println ("CAN Receiver Ready");
    }

    // Wait 10 seconds to allow for setup completion
    delay(8000);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    Serial.println ("Display Ready");

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(2000); // Pause for 2 seconds
    
    // Clear the buffer
    display.clearDisplay();
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
XBOX driver::getXboxData() {
    xboxController.onLoop(); // Process controller loop
    XBOX data; // Create an instance of the struct to hold the return values
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

                // Draw XBOX onto the display
                drawXBOX();
            }
            
            // Read and map joystick horizontal value to steering angle
            joyLHoriValue = (float)xboxController.xboxNotif.joyLHori;
            data.steeringAngle = map(joyLHoriValue, 0, 65535, 0 + steeringOffset, 180 - steeringOffset);

            // Read and map trigger values to throttle value
            float gas = (float)xboxController.xboxNotif.trigRT;
            float brake = (float)xboxController.xboxNotif.trigLT;
            data.throttleValue = map(gas - brake, -1023, 1023, 1000, 2000); // Map throttle value

            return data;
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

        data.steeringAngle = 90;
        data.throttleValue = 1500; // Map throttle value
        return data;
    }
}

CANBUS driver::getCanData() {

  CANBUS data; // Create an instance of the struct to hold the return values

  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // received a packet
    Serial.print ("Received ");

    if (CAN.packetExtended()) {
      Serial.print ("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print ("RTR ");
    }

    Serial.print ("packet with id 0x");
    Serial.print (CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print (" and requested length ");
      Serial.println (CAN.packetDlc());
    } else {
      Serial.print (" and length ");
      Serial.println (packetSize);

      // only print packet data for non-RTR packets
      while (CAN.available()) {
        Serial.print ((char) CAN.read());
        return data;
      }
      Serial.println();
    }

    Serial.println();
  }
}

void driver::driving(int driveMode){
  
  if (driveMode == 1){
    // Call the getXBOXdata function
    XBOX xboxData = getXboxData();

    // Access the returned throttleValue and steeringAngle
    float throttleValue = xboxData.throttleValue;
    float steeringAngle = xboxData.steeringAngle;
    
    // Center steering angle if within tolerance
    if (abs(steeringAngle - centerSteeringAngle) <= centerSteeringTolerance) {
      steeringAngle = centerSteeringAngle;
    }
    
    absimaServo.write(steeringAngle); // Set servo to steering angle
    absimaMotor.writeMicroseconds(throttleValue); // Set motor throttle
    drawValues(throttleValue, steeringAngle);
  }
}
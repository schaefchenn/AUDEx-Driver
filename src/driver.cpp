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
    CANBUS data;  // Create an instance of the struct to hold the return values
    
    int packetSize = CAN.parsePacket();
    
    if (packetSize) {
        // received a packet
        data.extended = CAN.packetExtended();
        data.rtr = CAN.packetRtr();
        data.id = CAN.packetId();
        data.length = CAN.packetDlc();
        
        // Read packet data into the struct
        for (int i = 0; i < packetSize; i++) {
            data.data[i] = CAN.read();
        }
        
        // Extract drive mode, throttle value, and steering angle from data array
        data.driveMode = data.data[0];  // Drive mode is the first byte
        data.throttleValue = data.data[1];  // Drive mode is the first byte
        data.steeringAngle = data.data[2];  // Steering angle is the third byte
    } else {
    
    }
    
    return data;
}

void driver::driving(int driveMode) {
    switch (driveMode) {
        case 1: {
            // Call the getXboxData function
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

            //Serial.print("\n Driving with XBOX");
            //Serial.print("\t Throttle: ");
            //Serial.print(throttleValue);
            //Serial.print("\t Steering: ");
            //Serial.print(steeringAngle);

            break;
        }
        case 2: {
            CANBUS canData = getCanData();
            
            // Access the returned throttleValue and steeringAngle
            uint8_t CANthrottleValue = map(canData.throttleValue, 100, 200, 1000, 2000);
            uint8_t CANsteeringAngle = canData.steeringAngle;

            // Center steering angle if within tolerance
            if (abs(CANsteeringAngle - centerSteeringAngle) <= centerSteeringTolerance) {
                CANsteeringAngle = centerSteeringAngle;
            }

            absimaServo.write(CANsteeringAngle); // Set servo to steering angle
            absimaMotor.writeMicroseconds(CANthrottleValue); // Set motor throttle
            drawValues(CANthrottleValue, CANsteeringAngle);

            //Serial.print("\n Driving with CAN");
            //Serial.print("\t Mode: ");
            //Serial.print(canData.driveMode);
            //Serial.print("\t Throttle: ");
            //Serial.print(CANthrottleValue);
            //Serial.print("\t Steering: ");
            //Serial.print(CANsteeringAngle);

            break;
        }
        default:
            Serial.print("\n Unknown drive mode");
            break;
    }
}
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
    absimaServo.setPeriodHertz(100);
    Serial.println("Steering Ready");

    // Motor setup - the Absima motor controller allows the motor to be treated as a servo
    absimaMotor.attach(motorPin);
    absimaMotor.setPeriodHertz(490);
    absimaMotor.writeMicroseconds(1500); // Neutral position for the motor
    Serial.println("Motor Ready");


    // Set CAN the pins
    CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

    // start the CAN bus at 1Mbps
    if (!CAN.begin (1E6)) {
        Serial.println ("Starting CAN failed!");
        while (1);
    } else {
        Serial.println ("CAN Ready");
    }


    if(rxMode == PPM_MODE){
        attachInterrupt(receiverPin, PPM_ISR, RISING);   // isr for measuring ppm signal from radio receiver
        Serial.println("PPM Receiver ready");
    }
    else if(rxMode == SBUS_MODE){
        sbusReceiver.begin(receiverPin, 5, true);
        Serial.println("SBUS Receiver ready");
    }

    
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_EXTERNALVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    Serial.println ("Display Ready");

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    // display.display();
    // delay(2000); // Pause for 2 seconds
    drawPorscheLogo();
    delay(2000); // Pause for 2 seconds
    // drawCat();
    // delay(4000);
    
    // Clear the buffer
    display.clearDisplay();
    Serial.println("Setup done");
}

// isr for reading ppm rx signal
void IRAM_ATTR driver::PPM_ISR(){
    static portMUX_TYPE _isr_mux = portMUX_INITIALIZER_UNLOCKED;
    static uint8_t ch_index = 0;
    static int64_t last_us;

    portENTER_CRITICAL_ISR(&_isr_mux);
    int64_t curr_us = esp_timer_get_time();
    int64_t ch_width = curr_us - last_us;
    last_us = curr_us;
    portEXIT_CRITICAL_ISR(&_isr_mux);

    if(ch_width > 3000 and ch_width < 12000) // sync
    {
        ch_index = 0;
        _ppm_data.available = false;
        if(_ppm_data.failsafe) _ppm_data.failsafe = false;
        return;
    }
    else if(ch_width > 12000)   // pulse width to long -> receiver not connected
    {   
        _ppm_data.failsafe = true;
        _ppm_data.available = false; 
        return;
    }

    if(ch_index < RX_MAX_CHANNELS)
        _ppm_data.channels[ch_index++] = constrain(ch_width, 1000, 2000);
    _ppm_data.available = true;
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

XBOX driver::getXboxData() {
    xboxController.onLoop(); // Process controller loop
    XBOX data; // Create an instance of the struct to hold the return values
    if (xboxController.isConnected()) { // Check if controller is connected
        // if (xboxController.isWaitingForFirstNotification()) {
        //     Serial.println("waiting for first notification");
        // } else {
            if (flag == 0) {
                demoVibration(); // Demonstrate vibration on first connection
                digitalWrite(LED_BUILTIN, HIGH); // Turn on built-in LED
                Serial.println("Address: " + xboxController.buildDeviceAddressStr()); // Print controller address
                Serial.println("battery " + String(xboxController.battery) + "%"); // Print battery status
                flag += 1;

                data.isConnected = true;
                driverReady = true;
                //sendCanData(driverReady);
            }

            // Read and map joystick horizontal value to steering angle
            joyLHoriValue = (float)xboxController.xboxNotif.joyLHori;
            data.steeringAngle = map(joyLHoriValue, 0, 65535, 0 + steeringOffset, 180 - steeringOffset);

            // Read and map trigger values to throttle value
            float gas = (float)xboxController.xboxNotif.trigRT;
            float brake = (float)xboxController.xboxNotif.trigLT;
            data.throttleValue = map(gas - brake, -1023, 1023, 1000, 2000); // Map throttle value

            // Read button states
            data.buttonA = xboxController.xboxNotif.btnA;
            data.buttonB = xboxController.xboxNotif.btnB;
            data.buttonX = xboxController.xboxNotif.btnX;
            data.buttonY = xboxController.xboxNotif.btnY;
            data.buttonLB = xboxController.xboxNotif.btnLB;
            data.buttonRB = xboxController.xboxNotif.btnRB;
            data.buttonStart = xboxController.xboxNotif.btnStart;
            data.buttonSelect = xboxController.xboxNotif.btnSelect;
            data.buttonLStick = xboxController.xboxNotif.btnLS;
            data.buttonRStick = xboxController.xboxNotif.btnRS;
            data.buttonCrossUP = xboxController.xboxNotif.btnRS;
            data.buttonCrossDOWN = xboxController.xboxNotif.btnRS;
            data.buttonCrossLEFT = xboxController.xboxNotif.btnRS;
            data.buttonCrossRIGHT = xboxController.xboxNotif.btnRS;

            data.isConnected = true;

            return data;
        // }
    } else {
        drawXBOXsearching();
        data.isConnected = false;
        driverReady = false;
        //delay(1000);
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
        data.acknowledged = data.data[3];  // Steering angle is the third byte
    } else {
    
    }
    
    return data;
}

void driver::sendCanData(int driverReady){
    CanMessage msg;
    msg.driverReady = driverReady;

    CAN.beginPacket(0x11);
    CAN.write((uint8_t*)&msg, sizeof(msg));
    CAN.endPacket();
}

bool driver::getPPMData(PPMData& data){
    for(byte i = 0; i < RX_MAX_CHANNELS; i++)
        data.channels[i] = _ppm_data.channels[i];
    return _ppm_data.available;
}

bool driver::getSbusData(SBUSData& data){
    if(sbusReceiver.readCal(data.channelsCal, &data.failSafe, &data.lostFrame)){
        for(byte i = 0; i < RX_MAX_CHANNELS; i++){
            data.channels[i] = data.channelsCal[i] * 1000 / 2 + 1500;    // convert channel values to 1000 - 2000
            data.channels[i] = constrain(data.channels[i], 1000, 2000);
        }
        return 1;
    }
    return 0;
}

Driver driver::driving(int driveMode, int CANthrottleValue, int CANsteerignAngle, int CANstatus, int CANflag) {
    Driver drivingData;
    drivingData.driveMode = driveMode;
    drivingData.CANflag = CANflag;
    drivingData.CANstatus = CANstatus;
    //Serial.println(drivingData.CANstatus);
    //Serial.print("\nButton Start pressed: ");
    //Serial.print(drivingData.CANflag);

    switch (driveMode) {
        case DRIVE_MODE_XBOX: {
            // Call the getXboxData function
            XBOX xboxData = getXboxData();
         
            if (xboxData.isConnected){

                // Access the returned throttleValue and steeringAngle
                float throttleValue = xboxData.throttleValue;
                float steeringAngle = xboxData.steeringAngle;

                // Center steering angle if within tolerance
                if (abs(steeringAngle - centerSteeringAngle) <= centerSteeringTolerance) {
                    steeringAngle = centerSteeringAngle;
                }

                absimaServo.write(steeringAngle); // Set servo to steering angle
                absimaMotor.writeMicroseconds(throttleValue); // Set motor throttle
                drawXBOXValues(throttleValue, steeringAngle, CANstatus);

                // if start button is presset set the CAN to pinging
                if (xboxData.buttonStart == 1){drivingData.CANflag = 0; delay(debounceDelay); }//delay for debounce

                if (xboxData.buttonSelect == 1){drivingData.driveMode = 3; delay(debounceDelay); }//delay for debounce
            }

            //Serial.print("\n Driving with XBOX");
            //Serial.print("\t Throttle: ");
            //Serial.print(throttleValue);
            //Serial.print("\t Steering: ");
            //Serial.print(steeringAngle);
            //Serial.print("\t Button A pressed: ");
            //Serial.print(xboxData.buttonA);
            //Serial.print("\t Button B pressed: ");
            //Serial.print(xboxData.buttonB);
            //Serial.print("\t Button Select pressed: ");
            //Serial.print(xboxData.buttonSelect);
            //Serial.print("\t Drive Mode: ");
            //Serial.print(driveMode);
            //Serial.print("\t Button Start pressed: ");
            //Serial.print(xboxData.buttonStart);
            //Serial.print("\t CAN flag: ");
            //Serial.print(drivingData.CANflag);
            //Serial.print("\t CAN status: ");
            //Serial.print(drivingData.CANstatus);

            break;
        }
        case DRIVE_MODE_CAN: {
           int steeringAngle = CANsteerignAngle;
           int throttleValue = CANthrottleValue *10;

            // Center steering angle if within tolerance
            if (abs(steeringAngle - centerSteeringAngle) <= centerSteeringTolerance) {
                steeringAngle = centerSteeringAngle;
            }

            absimaServo.write(steeringAngle); // Set servo to steering angle
            absimaMotor.writeMicroseconds(throttleValue); // Set motor throttle
            drawCANValues(throttleValue, steeringAngle);

            //Serial.print("\n Driving with CAN");
            //Serial.print("\t Mode: ");
            //Serial.print(driveMode);
            //Serial.print("\t Throttle: ");
            //Serial.print(throttleValue);
            //Serial.print("\t Steering: ");
            //Serial.print(steeringAngle);

            break;
        }
        case DRIVE_MODE_XBOX_LIMITED:{
            XBOX xboxData = getXboxData();
            if (xboxData.isConnected){

                if (xboxData.buttonA == 1 && throttleLimit < 2000){
                    throttleLimit += 100;
                    gear += 1;
                    delay(debounceDelay); //debounce
                }

                if (xboxData.buttonX == 1 && throttleLimit >= 1500){
                    throttleLimit -= 100;
                    gear -= 1;
                    delay(debounceDelay); 
                }

                // Access the returned throttleValue and steeringAngle
                float throttleValue = xboxData.throttleValue;
                if(throttleValue > throttleLimit && throttleLimit != 1400){
                    throttleValue = throttleLimit;
                } else if (throttleValue < 1500 && throttleLimit != 1400){
                    throttleValue = 1500;
                } else if (throttleLimit == 1400){
                    throttleValue = map(throttleValue,1000,2000, 2000, 1000);
                    if(throttleValue >1500){
                        throttleValue = 1500;
                    }
                }
               
                float steeringAngle = xboxData.steeringAngle;
                // Center steering angle if within tolerance
                if (abs(steeringAngle - centerSteeringAngle) <= centerSteeringTolerance) {
                    steeringAngle = centerSteeringAngle;
                }

                absimaServo.write(steeringAngle); // Set servo to steering angle
                absimaMotor.writeMicroseconds(throttleValue); // Set motor throttle

                absimaServo.write(steeringAngle); // Set servo to steering angle
                absimaMotor.writeMicroseconds(throttleValue); // Set motor throttle

                drawXBOXPitLimit(throttleValue, steeringAngle, throttleLimit, CANstatus );

                // if start button is presset set the CAN to pinging
                if (xboxData.buttonStart == 1){drivingData.CANflag = 0; delay(debounceDelay); }//delay for debounce

                if (xboxData.buttonSelect == 1){drivingData.driveMode = 4; throttleLimit=2000; delay(debounceDelay);} //debounce


                //Serial.print("\n Driving with XBOX Limiter");
                //Serial.print("\t Throttle: ");
                //Serial.print(throttleValue);
                //Serial.print("\t Throttle Limit: ");
                //Serial.print(throttleLimit);
                //Serial.print("\t Gear: ");
                //Serial.print(gear);
            }
            break;  
        }

        case DRIVE_MODE_RADIO_RX:{
            uint16_t throttle_us = 1500;
            uint16_t steering_us = 1500;
            bool data_available;
            bool failsafe;

            if(rxMode == PPM_MODE){
                PPMData ppm_data;
                data_available = getPPMData(ppm_data);
                failsafe = ppm_data.failsafe;
                // for(uint8_t i = 0; i < RX_MAX_CHANNELS; i++){
                //     Serial.print(ppm_data.channels[i]);
                //     Serial.print('\t');
                // }
                // Serial.println();
                throttle_us = ppm_data.channels[RX_THROTTLE_CH];
                steering_us = ppm_data.channels[RX_STEERING_CH];
            }
            else if(rxMode == SBUS_MODE){
                SBUSData sbus_data;
                data_available = getSbusData(sbus_data);
                failsafe = sbus_data.failSafe;
                // for(uint8_t i = 0; i < 8; i++){
                //     Serial.print(sbus_data.channels[i]);
                //     Serial.print('\t');
                // }
                // Serial.println();
                throttle_us = sbus_data.channels[RX_THROTTLE_CH];
                steering_us = sbus_data.channels[RX_STEERING_CH];
            }

            if(data_available){
                absimaMotor.writeMicroseconds(throttle_us); // Set motor throttle
                absimaServo.writeMicroseconds(steering_us); // Set servo to steering angle
                drawRXValues(throttle_us, steering_us, rxMode);
                // Serial.printf("throttle: %d, steering: %d\n", throttle_us, steering_us);
            }
            else if(failsafe) {
                absimaMotor.writeMicroseconds(1500); // Set motor throttle
                absimaServo.writeMicroseconds(1500); // Set servo to steering angle
            }
            
            break;
        }

        default:
            //Serial.print("\n Unknown drive mode");
            break;
    }

    return drivingData;
}
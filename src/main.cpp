#include <Arduino.h>
#include "driver.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// Declaring timer variables for task handling
unsigned long LoopCount1 = 0;
unsigned long LastLoopCountTime1 = 0;
unsigned long LoopCount2 = 0;
unsigned long LastLoopCountTime2 = 0;

int oldDriveMode = 1;
int currentDriveMode = 1;
int flag = 0;
bool resetEnabled = false;

// Initialize CPU cores
TaskHandle_t Task1;
TaskHandle_t Task2;

// Create an instance of the driver class
driver AUDEx;

// Function to send driver-ready notification and wait for acknowledgment
bool notifyDriverReady() {
    // Send driver-ready notification
    AUDEx.sendCanData(AUDEx.driverReady);
    Serial.println("Driver Ready notification sent");

    // Wait for acknowledgment with a timeout
    unsigned long startTime = millis();
    unsigned long timeout = 500; // half a second timeout
    while (millis() - startTime < timeout) {
        CANBUS canData = AUDEx.getCanData();
        if (canData.acknowledged == 1) {
            Serial.println("Acknowledgment received");
            return true;
        }
    }
    Serial.println("Acknowledgment not received, retrying...");
    return false;
}

// Code for CPU core 1 to sniff the CAN and switch driveMode to CAN driving if the CAN says so
void Task1code(void * pvParameters) {
    for (;;) {
        // to enable serial prints for second taks uncomment line below
        Serial.print("");
        if (AUDEx.driverReady) {
            if (flag == 0) {
                // Try notifying until acknowledgment is received
                while (!notifyDriverReady()) {
                    AUDEx.CANstatus = 1; // Driver CAN is pinging
                    delay(1000); // Wait 1 second before retrying
                }
                AUDEx.CANstatus = 2; // Driver CAN is receiving
                resetEnabled = true;
                flag += 1;
            }
            //Serial.print("Driver Ready");

            CANBUS canData = AUDEx.getCanData();
            currentDriveMode = canData.driveMode;

            AUDEx.CANsteerignAngle = canData.steeringAngle;
            AUDEx.CANthrottleValue = canData.throttleValue;

            //Serial.print("\t CAN Data on task 1");
            //Serial.print("\t Mode: ");
            //Serial.print(canData.driveMode);
            //Serial.print("\t Throttle: ");
            //Serial.print(AUDEx.CANthrottleValue);
            //Serial.print("\t Steering: ");
            //Serial.print(AUDEx.CANsteerignAngle);

            switch (currentDriveMode) {
                case 1:
                    if (AUDEx.driveMode == 2) {
                        // Transition logic from mode 1 to mode 2
                        AUDEx.driveMode = 1;
                        Serial.println("\nMode Switched to XBOX");
                    }
                    break;

                case 2:
                    if (AUDEx.driveMode == 1 or AUDEx.driveMode ==3 ) {
                        // Transition logic from mode 2 to another mode (if needed)
                        AUDEx.driveMode = 2;
                        Serial.println("\nMode Switched to CAN");
                    }
                    break;

                // Add more cases as needed

                default:
                    break;
            }

            #ifdef LoopCount
                LoopCount1++;
                if (LoopCount1 >= 10000) {
                    Serial.println("Core 1 10000 loops after: " + String(millis() - LastLoopCountTime1) + " ms");
                    LoopCount1 = 0;
                    LastLoopCountTime1 = millis();
                }
            #endif

            // Feed the watchdog timer
            TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
            TIMERG0.wdt_feed = 1;
            TIMERG0.wdt_wprotect = 0;
        }
    }
}

void Task2code(void * pvParameters) {
    for (;;) {
        Driver drivingData = AUDEx.driving(AUDEx.driveMode, AUDEx.CANthrottleValue, AUDEx.CANsteerignAngle, AUDEx.CANstatus, AUDEx.CANflag);
        AUDEx.driveMode = drivingData.driveMode;
        //flag = drivingData.CANflag;
        //if(resetEnabled){
            //flag = drivingData.CANflag;
        //}
        Serial.print("\nDrive Mode:");
        Serial.print(AUDEx.driveMode);

        //Serial.println("looping lets goooooooooooooooo!");

        #ifdef LoopCount
            LoopCount2++;
            if (LoopCount2 >= 10000) {
                Serial.println("Core 2 10000 loops after: " + String(millis() - LastLoopCountTime2) + " ms");
                LoopCount2 = 0;
                LastLoopCountTime2 = millis();
            }
        #endif

        // Feed the watchdog timer
        TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
        TIMERG0.wdt_feed = 1;
        TIMERG0.wdt_wprotect = 0;
    }
}

void setup() {
    // Initialize serial communication at 115200 baud rate
    Serial.begin(115200);

    // Initializing the two CPU Cores
    xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
    delay(500);

    xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
    delay(500);

    // Call the setup function of the driver instance
    AUDEx.setup();
    AUDEx.driveMode = 1; // 1 is for driving with the XBOX Series X Controller
}

void loop() {
    // Feed the watchdog timer
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;

    while (true) { }
}

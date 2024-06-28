#include <Arduino.h>
#include "driver.h"

// Core definitions (assuming you have dual-core ESP32)
static const BaseType_t pro_cpu = 0; // protocol core
static const BaseType_t app_cpu = 1; // application core

// Globals
static int oldDriveMode = 1;
static int currentDriveMode = 1;
static int flag = 0;

SemaphoreHandle_t canFlagMutex;
SemaphoreHandle_t driveModeMutex;

// Initialize CPU cores
TaskHandle_t Task1;
TaskHandle_t Task2;

float throttleValue;

// Create an instance of the driver class
driver AUDEx;

//*****************************************************************************
// Functions

// Function to send driver-ready notification and wait for acknowledgment
bool notifyDriverReady() {
    // Send driver-ready notification
    AUDEx.sendCanData(AUDEx.driverReady, 150);

    // Wait for acknowledgment with a timeout
    unsigned long startTime = millis();
    unsigned long timeout = 500; // half a second timeout
    while (millis() - startTime < timeout) {
        CANBUS canData = AUDEx.getCanData();
        if (canData.acknowledged == 1) {
            Serial.println("\tAcknowledgment received");
            return true;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Add delay to yield
    }
    Serial.println("\tAcknowledgment not received, retrying...");
    return false;
}

//*****************************************************************************
// Tasks

void CANcommunication(void * pvParameters) {
    while (1) {
        if (AUDEx.driverReady) {
            if (flag == 0) {
                if (xSemaphoreTake(canFlagMutex, portMAX_DELAY) == pdTRUE) {
                    AUDEx.CANstatus = 1;
                    while (!notifyDriverReady()) {
                        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait 1 second before retrying
                    }

                    // Critical section
                    flag = 1;
                    AUDEx.CANstatus = 2;
                    Serial.println("Flag set to 1");
                    
                    // Give mutex after critical section
                    xSemaphoreGive(canFlagMutex);
                    
                    // Delay after critical section
                    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
                } else {
                    // Do something else
                    Serial.println("CAN communication is waiting for mutex ");
                    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
                }
            }

            CANBUS canData = AUDEx.getCanData();

            currentDriveMode = canData.driveMode;
            AUDEx.CANsteerignAngle = canData.steeringAngle;
            AUDEx.CANthrottleValue = canData.throttleValue;

            if (xSemaphoreTake(driveModeMutex, portMAX_DELAY) == pdTRUE) {
                // Critical section
                switch (currentDriveMode) {
                case 1:
                    if (AUDEx.driveMode == 2) {
                        AUDEx.driveMode = 1;
                        //Serial.println("Mode Switched to XBOX");
                    }
                    break;

                case 2:
                    if (AUDEx.driveMode == 1 || AUDEx.driveMode == 3) {
                        AUDEx.driveMode = 2;
                        //Serial.println("Mode Switched to CAN");
                    }
                    break;

                default:
                    break;
                }

                // Give mutex after critical section
                xSemaphoreGive(driveModeMutex);

                Serial.println(throttleValue/10);
                AUDEx.sendCanData(1, throttleValue/10);

                // Delay after critical section
                vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
            } else {
                // Do something else
                Serial.println("CAN communication is waiting for mutex ");
                vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
            }
            vTaskDelay(10 / portTICK_PERIOD_MS); // Add delay to yield
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Ensure the loop yields
    }
}

void vehicleControl(void * pvParameters) {
    while (1) {
        Driver drivingData = AUDEx.driving(AUDEx.driveMode, AUDEx.CANthrottleValue, AUDEx.CANsteerignAngle, AUDEx.CANstatus, AUDEx.CANflag);
        throttleValue = drivingData.throttleValue;

        if (xSemaphoreTake(driveModeMutex, portMAX_DELAY) == pdTRUE) {
            // Critical section
            AUDEx.driveMode = drivingData.driveMode;
            // Give mutex after critical section
            xSemaphoreGive(driveModeMutex);

            // Delay after critical section
            vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
        } else {
            // Do something else
            Serial.println("CAN communication is waiting for mutex ");
            vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
        }

        AUDEx.CANflag = drivingData.CANflag;
        if (AUDEx.CANflag == 0) {
            if (xSemaphoreTake(canFlagMutex, portMAX_DELAY) == pdTRUE) {
                // Critical section
                flag = 0;
                AUDEx.CANflag = 1;
                
                Serial.println("Flag set to 0");
                // Give mutex after critical section
                xSemaphoreGive(canFlagMutex);

                // Delay after critical section
                vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
            } else {
                // Do something else
                Serial.println("CAN communication is waiting for mutex ");
                vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Ensure the loop yields
    }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
    // Initialize serial communication at 115200 baud rate
    Serial.begin(115200);

    // Wait a moment to start (so we don't miss Serial output)
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Create mutex before starting tasks
    canFlagMutex = xSemaphoreCreateMutex();
    driveModeMutex = xSemaphoreCreateMutex();

    AUDEx.driveMode = DRIVE_MODE_XBOX;    // default drive mode on reboot
    // Call the setup function of the driver instance
    AUDEx.setup();

    // Start CANcommunication (priority set to 1, 0 is the lowest priority)
    xTaskCreatePinnedToCore(CANcommunication,       // Function to be called
                            "CANcommunication",     // Name of task
                            8192,                   // Increased stack size
                            NULL,                   // Parameter to pass to function
                            2,                      // Increased priority
                            NULL,                   // Task handle
                            pro_cpu);               // Assign to protocol core

    // Start driving (higher priority)
    xTaskCreatePinnedToCore(vehicleControl,
                            "driving",
                            8192,                   // Increased stack size
                            NULL,
                            3,                      // Increased priority
                            NULL,
                            app_cpu);               // Assign to application core

    // Delete "setup and loop" task
    vTaskDelete(NULL);
}

void loop() {
    // Execution should never get here
}

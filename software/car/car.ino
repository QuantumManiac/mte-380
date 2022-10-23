#include "Wire.h"

#include "libs/imu.cpp"
#include "libs/motors.cpp"
#include "libs/ultrasonic.cpp"

#define START_BUTTON_PIN 32

IMU imu;
Ultrasonic ultrasonic;
Motors motors;

unsigned long lastSensorPrint = 0;

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    imu.initialize();
    ultrasonic.initialize();
    motors.initialize();
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);

    while (digitalRead(START_BUTTON_PIN) == HIGH); // Wait until start button is pressed
    delay(5000); // When start button is pressed, wait 5 secs before starting
}

void loop()
{
    imu.updateIMUState();
    if (millis() - lastSensorPrint > 1000) {
        lastSensorPrint = millis();
        printSensorData();
    }

    processCommand();    
}

void printSensorData() {
    Serial.println("Pitch: " + String(imu.getIMUData().pitch) + " Yaw: " + String(imu.getIMUData().yaw) + " Roll: " + String(imu.getIMUData().roll));
    Serial.println("Distance: " + String(ultrasonic.getDist()));
}

void processCommand() {
    char command = Serial.read();
    // q - forward max
    // a - forward half
    // w - backward max
    // s - backward half
    // x - stop

    if (command != -1) {
        Serial.println(command);
        if (command == 'q') { 
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors.setMotorDirection(Motor(i), forward);
                motors.setMotorPower(Motor(i), 1);
            }
        } else if (command == 'a') {
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors.setMotorDirection(Motor(i), forward);
                motors.setMotorPower(Motor(i), 0.5);
            }
        } else if (command == 'w') {
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors.setMotorDirection(Motor(i), backward);
                motors.setMotorPower(Motor(i), 1);
            }
        } else if (command == 's') {
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors.setMotorDirection(Motor(i), backward);
                motors.setMotorPower(Motor(i), 0.5);
            }
        } else if (command == 'x') {
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors.setMotorPower(Motor(i), 0);
            }
        }
    }
}

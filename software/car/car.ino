#include "Wire.h"

#include "libs/imu.cpp"
#include "libs/motors.cpp"
#include "libs/ultrasonic.cpp"

#define START_BUTTON_PIN 32

IMU imu;
Ultrasonic ultrasonic;
Motors motors;

const int NUMTURNS = 10;

float distToTurn[NUMTURNS] = {};

float leftSpeed = 0.9, rightSpeed = 0.9, leftTurn = 0.9, rightTurn = -0.9, adjustment = 0.1;

unsigned long lastSensorPrint = 0;

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    imu.initialize();
    ultrasonic.initialize();
    motors.initialize();
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    Serial.println("Waiting for start button press");
    while (digitalRead(START_BUTTON_PIN) == HIGH); // Wait until start button is pressed
    Serial.println("Start button pressed");
    delay(5000); // When start button is pressed, wait 5 secs before starting
}

void runMotors(float left, float right) {
    // check direction
    MotorDirection dirLeft = (left < 1 ? backward : forward);
    MotorDirection dirRight = (right < 1 ? backward : forward);

	//start left motor
	motors.setMotorDirection(Motor(0), dirLeft);
	motors.setMotorPower(Motor(0), abs(left));
    
	motors.setMotorDirection(Motor(1), dirLeft);
	motors.setMotorPower(Motor(1), abs(left));
	//start right motor
	motors.setMotorDirection(Motor(2), dirRight);
	motors.setMotorPower(Motor(2), abs(right));
    
	motors.setMotorDirection(Motor(2), dirRight);
	motors.setMotorPower(Motor(2), abs(right));
}

void turn(int turnNum) {
	if (ultrasonic.getDist() < distToTurn[turnNum]) {
		//stop motors
		runMotors(leftTurn, rightTurn);
	}

	while (imu.getIMUData().yaw < 90) {
	}

	if (imu.getIMUData().yaw > 90) {
		runMotors(0, 0);
	}

}

void adjustWheels(int val) {
	if (val > 0) {
		runMotors(leftSpeed, rightSpeed + adjustment);
	}
	else if (val < 0) {
		runMotors(leftSpeed + adjustment, rightSpeed);
	}
}

void loop()
{
    imu.updateIMUState();
    if (millis() - lastSensorPrint > 1000) {
        lastSensorPrint = millis();
        printSensorData();
    }

    for (int i = 0; i < NUMTURNS; i ++) {
		//start motors
		runMotors(leftSpeed, rightSpeed);
		while (ultrasonic.getDist() > distToTurn[i]) {
            imu.updateIMUState();
            if (millis() - lastSensorPrint > 1000) {
                lastSensorPrint = millis();
            }
			//adjustThread.check();
		}
		turn(i);
	}
}

void printSensorData() {
    Serial.println("Pitch: " + String(imu.getIMUData().pitch) + " Yaw: " + String(imu.getIMUData().yaw) + " Roll: " + String(imu.getIMUData().roll));
    Serial.println("Distance: " + String(ultrasonic.getDist()));
}

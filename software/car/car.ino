#include "Wire.h"

#include "libs/imu.cpp"
#include "libs/motors.cpp"
#include "libs/ultrasonic.cpp"

#define START_BUTTON_PIN 32

IMU imu;
Ultrasonic ultrasonic;
Motors motors;

const int NUMTURNS = 11;
const float MINDIST = 5;
const float MAX_PITCH = 10;

float distToTurn[NUMTURNS] = {13, 13, 13, 43, 43, 43, 43, 73, 73, 73, 73};

float leftSpeed = 0.9, rightSpeed = 0.9, leftTurn = 0.9, rightTurn = -0.9, adjustment = 0.1;

unsigned long lastSensorPrint = 0;
float distanceToWall = 0;
int usState = 0;
float lastUSValue = 0;

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
    MotorDirection dirLeft = (left < 0 ? backward : forward);
    MotorDirection dirRight = (right < 0 ? backward : forward);

	//start left motor
	motors.setMotorDirection(Motor(0), dirLeft);
	motors.setMotorPower(Motor(0), abs(left));
    
	motors.setMotorDirection(Motor(1), dirLeft);
	motors.setMotorPower(Motor(1), abs(left));
	//start right motor
	motors.setMotorDirection(Motor(2), dirRight);
	motors.setMotorPower(Motor(2), abs(right));
    
	motors.setMotorDirection(Motor(3), dirRight);
	motors.setMotorPower(Motor(3), abs(right));
}

void turn(int turnNum) {
	if (ultrasonic.getDist() < distToTurn[turnNum]) {
		//stop motors
		runMotors(leftTurn, rightTurn);
	}

	float currentAngle = imu.getIMUData().yaw;
	while (imu.getIMUData().yaw < (currentAngle + 90)) {
		imu.updateIMUState();
        if (millis() - lastSensorPrint > 1000) {
            lastSensorPrint = millis();
        }
	}

	runMotors(0, 0);

}

void adjustWheels(int val) {
	if (val > 0) {
		runMotors(leftSpeed, rightSpeed + adjustment);
	}
	else if (val < 0) {
		runMotors(leftSpeed + adjustment, rightSpeed);
	}
}

void checkDistance() {
	if ((usState == 0) && ((lastUSValue - MINDIST) > distanceToWall)) {
		usState = 1;
		distanceToWall = lastUSValue;
	} else if ((usState == 1) && ((lastUSValue - MINDIST) < distanceToWall)) {
		usState = 0;
		lastUSValue = ultrasonic.getDist();
	}

	if ((usState == 0) && ((lastUSValue - MINDIST) < distanceToWall)) {
		lastUSValue = ultrasonic.getDist();
	}

	if ((usState == 1) && ((lastUSValue - MINDIST) > distanceToWall)) {
		distanceToWall = lastUSValue;
	}
}

void loop()
{
    imu.updateIMUState();
    if (millis() - lastSensorPrint > 1000) {
        lastSensorPrint = millis();
		lastUSValue = ultrasonic.getDist();
        printSensorData();
    }

	delay(10000);

	imu.setZeroes(true, true, true);

    for (int i = 0; i < NUMTURNS; i ++) {
		distanceToWall = ultrasonic.getDist();
		//start motors
		runMotors(leftSpeed, rightSpeed);
		while (distanceToWall > distToTurn[i]) {
            imu.updateIMUState();
            if (millis() - lastSensorPrint > 1000) {
                lastSensorPrint = millis();
            }
			if (imu.getIMUData().pitch < abs(MAX_PITCH))
				distanceToWall = ultrasonic.getDist();
			//checkDistance();
			//adjustThread.check();
		}
		turn(i);
	}
}


void printSensorData() {
    Serial.println("Pitch: " + String(imu.getIMUData().pitch) + " Yaw: " + String(imu.getIMUData().yaw) + " Roll: " + String(imu.getIMUData().roll));
    Serial.println("Distance: " + String(ultrasonic.getDist()));
}

/*
void loop()
{
    imu.updateIMUState();
    if (millis() - lastSensorPrint > 1000) {
        lastSensorPrint = millis();
		lastUSValue = ultrasonic.getDist();
        printSensorData();
    }

    runMotors(leftSpeed, rightSpeed);
	while (distanceToWall > distToTurn[i]) {
            imu.updateIMUState();
            if (millis() - lastSensorPrint > 1000) {
                lastSensorPrint = millis();
            }
			distanceToWall = ultrasonic.getDist();
			checkDistance();
			//adjustThread.check();
		}
}
*/
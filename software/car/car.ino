#include "Wire.h"

#include "libs/imu.cpp"
#include "libs/motors.cpp"
#include "libs/ultrasonic.cpp"

#define START_BUTTON_PIN 53

IMU imu;
Ultrasonic ultrasonic;
Motors motors;

const int NUMTURNS = 11;
const float MINDIST = 5;
const float MAX_PITCH = 30;
const float MAX_DIFF = 1;
const float MAX_OVERSHOOT = 40;
const float TURN_DIST_ADJUST = -2;

float distToTurn[NUMTURNS] = {23, 23, 23, 50, 50, 50, 50, 78, 78, 78, 78};

float leftSpeed = 0.8, rightSpeed = 0.8, leftAdjustL = -0.5, leftAdjustR = 0.5, rightAdjustL = 0.5, rightAdjustR = -0.5, rightTurn = -0.8, leftTurn = -0.8;

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
    //Serial.println("Waiting for start button press");
    while (digitalRead(START_BUTTON_PIN) == HIGH); // Wait until start button is pressed
    //Serial.println("Start button pressed");
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

float turnToZero (float turnAngle) {
	runMotors(leftSpeed, rightTurn);
	if (turnAngle >= 360) {
		// turn until gyro value goes from close to 360 --> something greater than 0
		while (imu.getIMUData().yaw < turnAngle && imu.getIMUData().yaw > MAX_OVERSHOOT) {
			imu.updateIMUState();
        	if (millis() - lastSensorPrint > 1000) {
            	lastSensorPrint = millis();
        	}
		}
		turnAngle = turnAngle - 360;
	}
	/*
	// complete the rest of the turn or complete the entire turn
	while (abs(imu.getIMUData().yaw - turnAngle) > 10) {
		imu.updateIMUState();
		if (millis() - lastSensorPrint > 1000) {
			lastSensorPrint = millis();
		}
		//Serial.println("turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
	}
	*/
	//runMotors(0, 0);
	//Serial.println("turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
	return turnAngle;
}

void turn(float currentAngle) {
	// stop motors
	//runMotors(0, 0);

	float turnAngle = currentAngle + 90;
	if (turnAngle >= 360)
		// simplify turning conditions
		turnAngle = turnToZero(turnAngle);

	while (abs(turnAngle - imu.getIMUData().yaw) > MAX_DIFF) { // TODO: MAKE THE 1 A CONSTANT
		imu.updateIMUState();
		if (millis() - lastSensorPrint > 1000) {
			lastSensorPrint = millis();
		}
		adjustWheels(turnAngle);
	}
}

void adjustWheels(float turnAngle) {
	// turn left and turn right
	runMotors(0, 0);
	
	// turn left
	while ((abs(turnAngle - imu.getIMUData().yaw) > MAX_DIFF && (turnAngle < imu.getIMUData().yaw) && (imu.getIMUData().yaw - turnAngle <= 180)) || (turnAngle > imu.getIMUData().yaw && (imu.getIMUData().yaw -360 < -350) && (turnAngle > 340))) {
		imu.updateIMUState();
		if (millis() - lastSensorPrint > 1000) {
			lastSensorPrint = millis();
		}
		if (abs(turnAngle - imu.getIMUData().yaw) > 20) {
			runMotors(leftTurn, rightSpeed);
		} else {
			runMotors(leftAdjustL, leftAdjustR);
		}
		//Serial.println("left adjust turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
	}

	runMotors(0, 0);
	// OR the turn angle + 360 - yaw is greater than the max diff
	// ex if the turn angle is 20 and yaw is 355, 380, turn right
	while ((turnAngle > imu.getIMUData().yaw && (abs(turnAngle - imu.getIMUData().yaw) > MAX_DIFF)) && (turnAngle - imu.getIMUData().yaw < 180) || (turnAngle < imu.getIMUData().yaw && (imu.getIMUData().yaw - 360 > -10))) {
		imu.updateIMUState();
		if (millis() - lastSensorPrint > 1000) {
			lastSensorPrint = millis();
		}
		if (abs(turnAngle - imu.getIMUData().yaw) > 20) {
			runMotors(leftSpeed, rightTurn);
		} else {
			runMotors(rightAdjustL, rightAdjustR);
		}
		//Serial.println("right adjust turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
	}
		
	runMotors(0, 0);
}

// MAIN
void loop()
{
    imu.updateIMUState();
    if (millis() - lastSensorPrint > 1000) {
        lastSensorPrint = millis();
		lastUSValue = ultrasonic.getDist();
        printSensorData();
    }

	delay(10000);
	//Serial.println("drive");
	imu.setZeroes(true, true, true);

    for (int i = 0; i < NUMTURNS; i ++) {
		float currentAngle = imu.getIMUData().yaw;
		distanceToWall = ultrasonic.getDist();
		//start motors
		runMotors(leftSpeed, rightSpeed);
		while (ultrasonic.getDist() > (distToTurn[i] + TURN_DIST_ADJUST)) {
            imu.updateIMUState();
            if (millis() - lastSensorPrint > 1000) {
                lastSensorPrint = millis();
				//printSensorData();
            }
			if ((abs(imu.getIMUData().pitch - 360) < abs(MAX_PITCH)) || (abs(imu.getIMUData().pitch) < abs(MAX_PITCH))) {
				distanceToWall = ultrasonic.getDist();
				printSensorData();
			}

			//adjustThread.check();
		}
		//Serial.println(i);
		turn(currentAngle);
	}
}


void printSensorData() {
    //Serial.println("Pitch: " + String(imu.getIMUData().pitch) + " Yaw: " + String(imu.getIMUData().yaw) + " Roll: " + String(imu.getIMUData().roll));
    //Serial.println("Distance: " + String(ultrasonic.getDist()));
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
#include "Wire.h"

#include "libraries/user/imu.cpp"
#include "libraries/user/motors.cpp"
#include "libraries/user/tof.cpp"

#define START_BUTTON_PIN 53
#define LOGGING_ENABLED true

IMU imu;
ToF tof;
Motors motors;

//PID constants
double kp = 10;
double ki = 0;
double kd = 1000;
 
unsigned long currentTime, previousTimeTurn, previousTimeStop;
double elapsedTimeTurn, elapsedTimeStop;
double errorTurn, errorStop;
double lastErrorTurn, lastErrorStop;
double input, output, setPoint;
double cumError, rateError;
float currentMotorSpeedL, currentMotorSpeedR;
float MAX_SPEED = 1, MIN_SPEED = -1;

const int NUMTURNS = 11;
const float MINDIST = 5;
const float MAX_PITCH = 4;
const float MAX_DIFF = 1;
const float MAX_OVERSHOOT = 40;
const float TURN_DIST_ADJUST = 2;

float distToTurn[NUMTURNS] = {23, 23, 23, 50, 50, 50, 50, 78, 78, 78, 78};

//  change back to 0.8 after
float leftSpeed = 0.8, rightSpeed = 0.8, leftAdjustL = -0.6, leftAdjustR = 0.6, rightAdjustL = 0.6, rightAdjustR = -0.6, rightTurn = -0.8, leftTurn = -0.8;

unsigned long lastSensorPrint = 0;
float distanceToWall = 0;
int usState = 0;
float lastUSValue = 0;


void setup()
{
    Wire.begin();
    Serial.begin(115200);
    imu.initialize();
	//writeToSerial("Init");
    tof.initialize();
	//writeToSerial("Init Done");
    motors.initialize();
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    writeToSerial("Waiting for start button press");
    while (digitalRead(START_BUTTON_PIN) == HIGH); // Wait until start button is pressed
    writeToSerial("Start button pressed");
    // delay(5000); // When start button is pressed, wait 5 secs before starting
}

void loop()
{
    imu.updateIMUState();
    if (millis() - lastSensorPrint > 1000) {
        lastSensorPrint = millis();
		//lastUSValue = tof.getDist();
        printSensorData();
    }

	delay(10000);


	//writeToSerial("drive");
	imu.setZeroes(true, true, true);

    for (int i = 0; i < NUMTURNS; i ++) {
		float currentAngle = imu.getIMUData().yaw;
		distanceToWall = tof.getDist();
		//start motors
		runMotors(leftSpeed, rightSpeed);

		printSensorData();
		while (tof.getDist() > (distToTurn[i] + TURN_DIST_ADJUST)) {
            imu.updateIMUState();
            if (millis() - lastSensorPrint > 1000) {
                lastSensorPrint = millis();
				//printSensorData();
            }
			if ((abs(imu.getIMUData().pitch - 360) < abs(MAX_PITCH)) || (abs(imu.getIMUData().pitch) < abs(MAX_PITCH))) {
				distanceToWall = tof.getDist();
				printSensorData();
			}

			//adjustThread.check();
		} 

		// Increase turn angle by 90 each junction
		turn((i+1)*90%360);

	}
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
	return turnAngle;
}

void turn(float currentAngle) {
	float turnAngle = currentAngle;
	if (turnAngle >= 360)
		// simplify turning conditions
		turnAngle = turnToZero(turnAngle);
	float currentTime = millis();

	while (abs(turnAngle - imu.getIMUData().yaw) > MAX_DIFF || millis() - currentTime < 2500) { // TODO: MAKE THE 1 A CONSTANT
		
		imu.updateIMUState();

		//adjustWheels(turnAngle);
		float turning = computePIDTurn(turnAngle, imu.getIMUData().yaw);
		if (turning > 1) {
			runMotors(1, -1);
		} else if (turning < -1) {
			runMotors(-1, 1);
		} else {
			runMotors(turning, turning*-1);
		} 
		if (millis() - lastSensorPrint > 200) {
			lastSensorPrint = millis();
			writeToSerial(String(millis()) + " adjust turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
			writeToSerial("PID Value: " + String(turning));
		}
	}
	motors.brakeAllMotors();
}

void adjustWheels(float turnAngle) {
	
	motors.brakeAllMotors();

	// OR the turn angle + 360 - yaw is greater than the max diff
	// ex if the turn angle is 20 and yaw is 355, 380, turn right
	while ((turnAngle > imu.getIMUData().yaw && (abs(turnAngle - imu.getIMUData().yaw) > MAX_DIFF)) && (turnAngle - imu.getIMUData().yaw < 180) || (turnAngle < imu.getIMUData().yaw && (imu.getIMUData().yaw - 360 > -20))) {
		imu.updateIMUState();
		if (millis() - lastSensorPrint > 1000) {
			lastSensorPrint = millis();
		}
		if (abs(turnAngle - imu.getIMUData().yaw) > 20) {
			runMotors(0.7, -0.7);
		} else {
			runMotors(0.4, -0.4);
		}
		//writeToSerial(String(millis()) + " right adjust turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
	}
	// turn left and turn right
	motors.brakeAllMotors();
	
	// turn left
	while ((abs(turnAngle - imu.getIMUData().yaw) > MAX_DIFF && (turnAngle < imu.getIMUData().yaw) && (imu.getIMUData().yaw - turnAngle <= 180)) || (turnAngle > imu.getIMUData().yaw && (imu.getIMUData().yaw -360 < -340) && (turnAngle > 340))) {
		imu.updateIMUState();
		if (millis() - lastSensorPrint > 1000) {
			lastSensorPrint = millis();
		}
		if (abs(turnAngle - imu.getIMUData().yaw) > 20) {
			runMotors(-0.7, 0.7);
		} else {
			runMotors(-0.4, 0.4);
		}
		//writeToSerial(String(millis()) + " left adjust turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
	}
		
	//runMotors(0, 0);
	motors.brakeAllMotors();
	printSensorData();
}

float computePIDTurn(double setPoint, double inp){     
	currentTime = millis();                //get current time
	// generally 4-10 ms
	elapsedTimeTurn = (double)(currentTime - previousTimeTurn);        //compute time elapsed from previous computation
	
	if (setPoint < inp && inp > 340 && setPoint < 91) {
		// should turn right therefore positive error
		errorTurn = 360 - inp + setPoint;
	} else if (inp < setPoint && inp < 20 && setPoint > 269) {
		// should turn left therefore negative error
		errorTurn = setPoint + inp - 360;
	} else {
		errorTurn = setPoint - inp; // determine error
	}
	cumError += errorTurn * elapsedTimeTurn;                // compute integral
	writeToSerial("Elapsed Time: " + String(elapsedTimeTurn));
	rateError = (errorTurn - lastErrorTurn)/elapsedTimeTurn;   // compute derivative

	double out = kp*errorTurn + ki*cumError + kd*rateError;                //PID output      
	if (out < -360) {
		out = -360;
	} else if (out > 360) {
		out = 360;
	}

	lastErrorTurn = errorTurn;                                //remember current error
	previousTimeTurn = currentTime;                        //remember current time
	out /= 360;
	return out;                                        //have function return the PID output
}

void printSensorData() {
    writeToSerial("Pitch: " + String(imu.getIMUData().pitch) + " Yaw: " + String(imu.getIMUData().yaw) + " Roll: " + String(imu.getIMUData().roll));
    writeToSerial("Distance: " + String(tof.getDist()));
    writeToSerial("Time: " + String(millis()));
}

void writeToSerial(String text) {
    if (LOGGING_ENABLED) {
        Serial.println(text);
    }
}
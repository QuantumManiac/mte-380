#include "Wire.h"

#include "libraries/user/imu.cpp"
#include "libraries/user/motors.cpp"
#include "libraries/user/tof.cpp"

#define START_BUTTON_PIN 53
#define TEST_DRIVE 0
#define TEST_TURN 0
#define TEST_SPEED 0

IMU imu;
ToF tof;
Motors motors;

//PID constants
double kp = 1;
double ki = 0;
double kd = 0;
 
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
	//Serial.println("Init");
    tof.initialize();
	//Serial.println("Init Done");
    motors.initialize();

    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    //Serial.println("Waiting for start button press");
    while (digitalRead(START_BUTTON_PIN) == HIGH); // Wait until start button is pressed
    //Serial.println("Start button pressed");
    // delay(5000); // When start button is pressed, wait 5 secs before starting
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
			Serial.println(String(millis()) + " adjust turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
			//Serial.println("Motor speeds: left | right: " + String(motors.motor_pwm_values[front_left]) + " | " + String(motors.motor_pwm_values[front_right]));
			Serial.println("PID Value: " + String(turning));
		}
	}
	motors.brakeAllMotors();

	Serial.println(String(millis() - currentTime));
}

void adjustWheels(float turnAngle) {
	
	//runMotors(0, 0);
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
		//Serial.println(String(millis()) + " right adjust turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
	}
	// turn left and turn right
	//runMotors(0, 0);
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
		//Serial.println(String(millis()) + " left adjust turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
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
	//Serial.println("Cumulative Error: " + String(cumError));
	//Serial.println("Turn Error: " + String(errorTurn));
	rateError = (errorTurn - lastErrorTurn)/elapsedTimeTurn;   // compute derivative

	double out = kp*errorTurn + ki*cumError + kd*rateError;  
	//PID output      
	if (out < -360) {
		out = -360;
	} else if (out > 360) {
		out = 360;
	}

	lastErrorTurn = errorTurn;                             //remember current error
	previousTimeTurn = currentTime;                        //remember current time

	//out = map(out, -360, 360, -1, 1);
	double minMotorSpeed = 0.4;
	out /= 360;
	if (out < minMotorSpeed && out > 0) {
		out = minMotorSpeed;
	} else if (out > -minMotorSpeed && out < 0) {
		out = -minMotorSpeed;
	}

	return out;                                        //have function return the PID output
}

// MAIN
void loop()
{
    imu.updateIMUState();
    if (millis() - lastSensorPrint > 1000) {
        lastSensorPrint = millis();
		//lastUSValue = tof.getDist();
        printSensorData();
    }

	//delay(10000);


	//Serial.println("drive");
	imu.setZeroes(true, true, true);

    for (int i = 0; i < NUMTURNS; i ++) {
		float currentAngle = imu.getIMUData().yaw;
		distanceToWall = tof.getDist();
		//start motors
		runMotors(leftSpeed, rightSpeed);

		printSensorData();
		/*
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
		*/

		// Increase turn angle by 90 each junction
		turn((i+1)*90%360);
		delay(1000);

	}
}


void printSensorData() {
    //Serial.println("Pitch: " + String(imu.getIMUData().pitch) + " Yaw: " + String(imu.getIMUData().yaw) + " Roll: " + String(imu.getIMUData().roll));
    //Serial.println("Distance: " + String(tof.getDist()));
    //Serial.println("Time: " + String(millis()));
}

/*
// pid implementation


//PID constants
double kp = 2;
double ki = 5;
double kd = 1;
 
unsigned long currentTime, previousTimeTurn, previousTimeStop;
double elapsedTimeTurn, elapsedTimeStop;
double errorTurn, errorStop;
double lastErrorTurn, lastErrorStop;
double input, output, setPoint;
double cumError, rateError;
float currentMotorSpeedL, currentMotorSpeedR;
float MAX_SPEED = 1, MIN_SPEED = -1;
 
void setup()
{ 
    Wire.begin();
    Serial.begin(115200);
    imu.initialize();
    tof.initialize();
    motors.initialize();
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    Serial.println("Waiting for start button press");
    while (digitalRead(START_BUTTON_PIN) == HIGH); // Wait until start button is pressed
    Serial.println("Start button pressed");
    // delay(5000); // When start button is pressed, wait 5 secs before starting
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

void loop(){
	setPoint = 0; // change setPoint to equal 0, 90, 180, 270
	input = imu.getIMUData().yaw;                //read from rotary encoder connected to A0
	output = computePIDTurn(setPoint, input);
	runMotors(0.8, 0.8);
	delay(100);
	while(tof.getDist() > 23) {
		input = imu.getIMUData().yaw; 
		output = computePIDTurn(setPoint, input);
		if (output > MAX_SPEED) {
			currentMotorSpeedR = MAX_SPEED;
			currentMotorSpeedL = 0;
		}
		if (output < MIN_SPEED) {
			currentMotorSpeedL = MAX_SPEED;
			currentMotorSpeedR = 0;
		} else if (output < 0) {
			currentMotorSpeedR = (-1*output) / 2;
			currentMotorSpeedL = output / 2;
		} else {
			currentMotorSpeedL = 0;
		}
		runMotors(output, -1*output);
	}             //control the motor based on PID value
 
}
 
double computePIDTurn(double setPoint, double inp){     
	currentTime = millis();                //get current time
	elapsedTimeTurn = (double)(currentTime - previousTimeTurn);        //compute time elapsed from previous computation
	
	if (setPoint < inp && inp > 180 && setPoint < 180) {
		errorTurn = inp - setPoint - 360;
	} else if (inp < setPoint && inp < 180 && setPoint > 180) {
		errorTurn = 360 - setPoint + inp;
	} else {
		errorTurn = setPoint - inp; // determine error
	}
	cumError += errorTurn * elapsedTimeTurn;                // compute integral
	rateError = (errorTurn - lastErrorTurn)/elapsedTimeTurn;   // compute derivative

	double out = kp*errorTurn + ki*cumError + kd*rateError;                //PID output               

	lastErrorTurn = errorTurn;                                //remember current error
	previousTimeTurn = currentTime;                        //remember current time

	out = map(out, -360, 360, -1, 1);

	return out;                                        //have function return the PID output
}

float computePIDStop(float setPoint, float inp) {
	currentTime = millis();                //get current time
	elapsedTimeStop = (double)(currentTime - previousTimeStop);        //compute time elapsed from previous computation
	
	errorStop = setPoint - inp; // determine error

	cumError += errorStop * elapsedTimeStop;                // compute integral
	rateError = (errorStop - lastErrorStop)/elapsedTimeStop;   // compute derivative

	double out = kp*errorStop + ki*cumError + kd*rateError;                //PID output               

	lastErrorStop = errorStop;                                //remember current error
	previousTimeStop = currentTime;                        //remember current time

	out = map(out, inp, setPoint, 0, 1);
	return out;  
}

*/

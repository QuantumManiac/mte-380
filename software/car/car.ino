#include "Wire.h"

#include "libraries/user/imu.cpp"
#include "libraries/user/motors.cpp"
#include "libraries/user/ultrasonic.cpp"

#define START_BUTTON_PIN 53

IMU imu;
Ultrasonic ultrasonic;
Motors motors;

const int NUMTURNS = 11;
const float MINDIST = 5;
const float MAX_PITCH = 4;
const float MAX_DIFF = 1;
const float MAX_OVERSHOOT = 40;
const float TURN_DIST_ADJUST = -2;

float distToTurn[NUMTURNS] = {23, 23, 23, 50, 50, 50, 50, 78, 78, 78, 78};

float leftSpeed = 0.8, rightSpeed = 0.8, leftAdjustL = -0.6, leftAdjustR = 0.6, rightAdjustL = 0.6, rightAdjustR = -0.6, rightTurn = -0.9, leftTurn = -0.9;

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
	while (true) {
		imu.updateIMUState();
		printSensorData();
	}
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

	while (abs(turnAngle - imu.getIMUData().yaw) > MAX_DIFF) { // TODO: MAKE THE 1 A CONSTANT
		imu.updateIMUState();
		if (millis() - lastSensorPrint > 1000) {
			lastSensorPrint = millis();
		}
		adjustWheels(turnAngle);
	}
}

void adjustWheels(float turnAngle) {
	
	runMotors(0, 0);
	// OR the turn angle + 360 - yaw is greater than the max diff
	// ex if the turn angle is 20 and yaw is 355, 380, turn right
	while ((turnAngle > imu.getIMUData().yaw && (abs(turnAngle - imu.getIMUData().yaw) > MAX_DIFF)) && (turnAngle - imu.getIMUData().yaw < 180) || (turnAngle < imu.getIMUData().yaw && (imu.getIMUData().yaw - 360 > -20))) {
		imu.updateIMUState();
		if (millis() - lastSensorPrint > 1000) {
			lastSensorPrint = millis();
		}
		if (abs(turnAngle - imu.getIMUData().yaw) > 20) {
			runMotors(leftSpeed, rightTurn);
		} else {
			runMotors(rightAdjustL, rightAdjustR);
		}
		Serial.println(String(millis()) + " right adjust turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
	}
	// turn left and turn right
	runMotors(0, 0);
	
	// turn left
	while ((abs(turnAngle - imu.getIMUData().yaw) > MAX_DIFF && (turnAngle < imu.getIMUData().yaw) && (imu.getIMUData().yaw - turnAngle <= 180)) || (turnAngle > imu.getIMUData().yaw && (imu.getIMUData().yaw -360 < -340) && (turnAngle > 340))) {
		imu.updateIMUState();
		if (millis() - lastSensorPrint > 1000) {
			lastSensorPrint = millis();
		}
		if (abs(turnAngle - imu.getIMUData().yaw) > 20) {
			runMotors(leftTurn, rightSpeed);
		} else {
			runMotors(leftAdjustL, leftAdjustR);
		}
		Serial.println(String(millis()) + " left adjust turnAngle: " + String(turnAngle) + " imu yaw: " + String(imu.getIMUData().yaw));
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
		Serial.println(i);
		
		// Increase turn angle by 90 each junction
		turn((i+1)*90%360);
	}
}

void printSensorData() {
    Serial.println("Pitch: " + String(imu.getIMUData().pitch) + " Yaw: " + String(imu.getIMUData().yaw) + " Roll: " + String(imu.getIMUData().roll));
    Serial.println("Distance: " + String(ultrasonic.getDist()));
    Serial.println("Time: " + String(millis()));
}

/*
// pid implementation


//PID constants
double kp = 2;
double ki = 5;
double kd = 1;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
double setPoint;
 
void setup(){
}    
 
void loop(){
		setPoint = 0; // change setPoint to equal 0, 90, 180, 270
        input = imu.getIMUDate().yaw;                //read from rotary encoder connected to A0
        output = computePID(setPoint, input);
        delay(100);
        analogWrite(3, output);                //control the motor based on PID value
 
}
 
double computePID(double setPoint, double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
		if (setPoint < inp && inp > 180 && setPoint < 180) {
			error = inp - setPoint - 360;
		} else if (inp < setPoint && inp < 180 && setPoint > 180) {
			error = 360 - setPoint + inp;
		} else {
			error = setPoint - inp; // determine error
		}
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

		out = map(out, 0, 360, -1, 1);
 
        return out;                                        //have function return the PID output
}
*/
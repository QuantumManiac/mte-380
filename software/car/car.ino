#include "Wire.h"
#include <PID_v1.h>
#include <sTune.h>

#include "libraries/user/imu.cpp"
#include "libraries/user/motors.cpp"
#include "libraries/user/tof.cpp"

#define START_BUTTON_PIN 53

#define SERIAL_LOGGING false   // set to false to prevent potential blocking of code
#define CALIBRATE_IMU true    // Enables wait for 10 seconds before starting
#define PRINT_SENSOR_DATA true // Requires SERIAL_LOGGING to be true

const int SENSOR_PRINT_INTERVAL = 300; // Interval to print sensor values using printSensorData (ms)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         RINT_INTERVAL = 250; // Interval to print sensor values using printSensorData (ms)

// Initialize objects
IMU imu;
ToF tof;
Motors motors;

const int NUM_TURNS = 11; // Number of turns to make in the course
const float MAX_DIFF = 2.5; // Threshold for difference between target and actual angle
const float MAX_PITCH = 6;
const float MIN_TIME = 1;
const float MIN_DIST_DIFF = 200;
const float MIN_PITCH = -30;
const float OUT_PIT = 0.6;
const unsigned long MAX_SETTLE_TIME = 0; // Max time given to PIDs to settle
const float TURN_ANGLE = 90.;
const float MAX_OVERSHOOT = 40.;
const float SPEED_DROP_DIST = 2;
const float TURN_DIST_BIAS = -50.; // to adjust steady state turn distance
const float TURN_STOP_BIAS = 75; // to adjust distance before stopping to adjust before turning
const float TURN_DIST_TOL = 35;
const float TURN_DIST_TIME = 1000;
const float STRAIGHT_TOL = 2;
const float distToTurn[NUM_TURNS] = {130., 130., 130., 400., 400., 400., 400., 680., 680., 680., 680.}; // Distances from wall (in mm) to turn at for every turn


// PID-related variables 
const int SAMPLE_TIME = 100; // Time between PID calculations (ms)
double turnKp = 0.1, turnKi = 0, turnKd = 0.003;
double straightKp = 250, straightKi = 0, straightKd = 10;
double turnInput, turnOutput; // Variables for turning PID control
double straightInput, straightOutput; // Variables for keeping straight PID control
double turnTarget = 0;
double straightTarget = 0;
PID turnPID(&turnInput, &turnOutput, &turnTarget, turnKp, turnKi, turnKd, DIRECT);
PID straightPID(&straightInput, &straightOutput, &straightTarget, straightKp, straightKi, straightKd, DIRECT);

//  Wheel speed variables
const float MIN_SPEED = 0.3;
const float CRUISE_SPEED = 0.45;
const float MIN_TURN_SPEED = 0.35;
const float MAX_TURN_SPEED = 0.55;

unsigned long lastSensorPrint = 0;
float distanceToWall = 0.;
int turnsDone = 0;

void setup()
{
    // Init I2C and Serial
    Wire.begin();
    Serial.begin(115200);

    // Init Sensors/Actuators
    imu.initialize();
    tof.initialize();
    motors.initialize();
    // Init PIDs
    turnPID.SetSampleTime(SAMPLE_TIME);
    straightPID.SetSampleTime(SAMPLE_TIME);
    turnPID.SetOutputLimits(-MAX_TURN_SPEED, MAX_TURN_SPEED); // Set output limits
    straightPID.SetOutputLimits(MIN_SPEED, CRUISE_SPEED);
    straightPID.SetMode(AUTOMATIC);

    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    printLineToSerial("Waiting for start button press"); // TODO: This should be in two stages for game day: press start button to go through calibration and/or pre-flight checks and then start again to go through the course
    while (digitalRead(START_BUTTON_PIN) == HIGH); // Wait until start button is pressed
    printLineToSerial("Start button pressed");

    imu.setZeroes(true, true, true);
#if CALIBRATE_IMU
    printLineToSerial("Calibrating IMU...");
    for (int i = 0; i < 15; i++)
    {
        printSensorData();
        tick();
        delay(1000);
        printLineToSerial("."); // Keep printing to serial to notify that stuff is still happening
    }
#endif
    tick();
    imu.setZeroes(true, true, true); // Zero out yaw, pitch, and roll at start
    printLineToSerial(String(imu.getIMUData().pitch) + " | " + String(imu.getIMUData().yaw)  + " | " + String(imu.getIMUData().roll));
}

void loop()
{
    // Continue straight to next turn
    runMotors(CRUISE_SPEED, CRUISE_SPEED);
    float prevDist = tof.getDist();
    float initialAngle = imu.getIMUData().yaw; // Turn 90 CW from at next turn

    straightPID.SetMode(AUTOMATIC); // re-init straightness PID

    while (tof.getDist() > (distToTurn[turnsDone] + TURN_STOP_BIAS) || inPit(prevDist, tof.getDist())) // While distance to wall is greater than the turning threshold
    {
        tick();
        #if PRINT_SENSOR_DATA
            // Sensor data printing routine
            if (millis() - lastSensorPrint > SENSOR_PRINT_INTERVAL)
            {
                lastSensorPrint = millis();
                printSensorData();
            }
        #endif
        if (abs(imu.getIMUData().pitch) < MAX_PITCH) { // if the pitch angle is greater than the maximum pitch of the course and the distance does not jump to a small number in a short period of time
            prevDist = tof.getDist();
        }
        // if its leaving the pit, REVERSE
        if (imu.getIMUData().pitch < -130) {
            printLineToSerial("Reversed: " + String(imu.getIMUData().pitch));
            runMotors(-CRUISE_SPEED, -CRUISE_SPEED);
            delay(1000);
        } else {
            runMotors(CRUISE_SPEED, CRUISE_SPEED);
        }
        
        if (tof.getDist() < (distToTurn[turnsDone] + distToTurn[turnsDone])) {
            printLineToSerial("Reduced Speed in loop");
            runMotors(MIN_SPEED, MIN_SPEED);
        }

        if(abs(imu.getIMUData().yaw - initialAngle) > STRAIGHT_TOL)
            adjustWheels(initialAngle); 
    }

    straightPID.SetMode(MANUAL);
    printLineToSerial("Stop Distance: " + String(tof.getDist()));
    wallStop();

    // TODO: Center in tile before turning
    turnCorner(initialAngle);
    turnsDone += 1;
    printLineToSerial("============!!!TURNS DONE!!!: " + String(turnsDone) + "======================");
    imu.addToOffsets(-90, 0, 0); // Subtract 90 degrees from yaw offset so we're still close to zero degree heading after the turn, but accounting for the error
    // imu.setZeroes(true, true, true); // Alternatively, we could just zero out the IMU after turning but that will accumulate error in a different way

    // Stop and wait when done course
    if (turnsDone == NUM_TURNS)
    {
        printLineToSerial("Done!");
        while (true);
    }
}

/**
 * @brief checks the distance to the wall
 * 
 */
void wallStop() {
    motors.brakeAllMotors();
    delay(100);
    printLineToSerial("Entered wall stop fxn at " + String(millis()));
    float savedTime = millis();
    while (abs(tof.getDist() - (distToTurn[turnsDone] + TURN_DIST_BIAS)) > TURN_DIST_TOL || ((millis() - savedTime) < TURN_DIST_TIME)) {
        // Move forward or backward to move to right distance
        if (tof.getDist() > (distToTurn[turnsDone] + TURN_DIST_BIAS)) {
            runMotors(0.25, 0.25);
        } else {
            runMotors(-0.25, -0.25);
        }
        if (abs(tof.getDist() - (distToTurn[turnsDone] + TURN_DIST_BIAS)) > TURN_DIST_TOL) {
            savedTime = millis();
        } else {
            motors.brakeAllMotors();
        }
    }
}

/**
 * @brief checks the difference between the current value to the wall and the previous value to the wall
 * 
 * @param prevDist previous distance to wall recorded
 * @param currDist current distance to wall recorded
 */
bool inPit(float prevDist, float currDist) {
    return (prevDist - currDist) > MIN_DIST_DIFF; // return the difference between the previous distance and the current distance
}

/**
 * @brief sets the left and right motor speeds and directioons
 * 
 * @param left power for left motors (0-1, negative for reverse)
 * @param right power for right motors (0-1, negative for reverse)
 */
void runMotors(float left, float right)
{
    // Set direction
    MotorDirection dirLeft = (left < 0 ? backward : forward);
    MotorDirection dirRight = (right < 0 ? backward : forward);

    // Set left motor direction and speed
    motors.setMotorDirection(front_left, dirLeft);
    motors.setMotorPower(front_left, abs(left));

    motors.setMotorDirection(back_left, dirLeft);
    motors.setMotorPower(back_left, abs(left));

    // Set right motor direction and speed
    motors.setMotorDirection(front_right, dirRight);
    motors.setMotorPower(front_right, abs(right));

    motors.setMotorDirection(back_right, dirRight);
    motors.setMotorPower(back_right, abs(right));
}

/**
 * @brief Turns the car 90 degrees when at a corner
 * 
 */
void turnCorner(float initialAngle)
{
    // Stop car
    motors.brakeAllMotors();
    // Enable PID
    turnPID.SetMode(AUTOMATIC); 
    // Record initial values
    unsigned long initialTime = 0;
    // Set target angle for PID
    turnTarget = initialAngle + TURN_ANGLE;
    // Keep turning until within threshold of target angle and enough time to settle has elapsed
    // TODO: settling time should start only once the robot reaches the target angle for the first time
    while ((abs((turnTarget) - imu.getIMUData().yaw) >= MAX_DIFF) ||  (initialTime == 0 || ((millis() - initialTime) < MAX_SETTLE_TIME)))
    { 
        // Set initial time for settle once target reached for first time
        if ((abs((TURN_ANGLE) - imu.getIMUData().yaw) <= MAX_DIFF) && initialTime == 0) {
            initialTime = millis();
        }
        
        tick();
        // Minimum turn speed to prevent stalling
        if (turnOutput > 0 && turnOutput < MIN_TURN_SPEED)
        {
            turnOutput = MIN_TURN_SPEED;
        }
        else if (turnOutput < 0 && turnOutput > -MIN_TURN_SPEED)
        {
            turnOutput = -MIN_TURN_SPEED;
        }
        
        runMotors(turnOutput, -turnOutput);

#if PRINT_SENSOR_DATA
        if (millis() - lastSensorPrint > 200)
        {
            lastSensorPrint = millis();
            printLineToSerial("Turn to target:" + String(TURN_ANGLE) + " within " + String(MAX_DIFF));
            printLineToSerial(String(millis()) + "imu yaw: " + String(imu.getIMUData().yaw));
            printLineToSerial("PID Output: " + String(turnOutput)); 
        }
#endif
    }
    motors.brakeAllMotors();
    // Disable PID
    turnPID.SetMode(MANUAL); 
}

void adjustWheels(float targetAngle)
{
    float speedScaling = 1;
    if (tof.getDist() < (distToTurn[turnsDone] + (distToTurn[turnsDone]))) {
            speedScaling = 0.67;
            printLineToSerial("Speed Scaling: " + String(speedScaling));
        }
    if (imu.getIMUData().yaw > targetAngle)
        runMotors(MIN_SPEED*speedScaling, (straightOutput+CRUISE_SPEED)*speedScaling);
    else
        runMotors((straightOutput+CRUISE_SPEED)*speedScaling, MIN_SPEED*speedScaling);
}   

/**
 * @brief Prints the sensor data to the serial monitor
 * 
 */
void printSensorData()
{
    printLineToSerial("Pitch: " + String(imu.getIMUData().pitch) + " Yaw: " + String(imu.getIMUData().yaw) + " Roll: " + String(imu.getIMUData().roll));
    printLineToSerial("Distance: " + String(tof.getDist()));
    printLineToSerial("Time: " + String(millis()));
}

/**
 * @brief Wrapper function that prints a line to the serial monitor. Used to disable serial printing using the SERIAL_LOGGING variable if required
 * 
 * @param line string to print to the serial monitor
 */
void printLineToSerial(String text)
{
    if (SERIAL_LOGGING)
    {
        Serial.println(text);
    }
}

/**
 * @brief Computes the values for all PIDs
 * 
 */
void computePIDs()
{
    straightPID.Compute();
    turnPID.Compute();
}

/**
 * @brief Runs functions that should be called every loop
 * 
 */
void tick()
{
    imu.updateIMUState();
    computePIDs();
    // Update current angle for PIDs
    turnInput = straightInput = imu.getIMUData().yaw;
}
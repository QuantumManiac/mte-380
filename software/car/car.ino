#include "Wire.h"
#include <PID_v1.h>

#include "libraries/user/imu.cpp"
#include "libraries/user/motors.cpp"
#include "libraries/user/tof.cpp"

#define START_BUTTON_PIN 53

#define SERIAL_LOGGING true    // set to false to prevent potential blocking of code
#define CALIBRATE_IMU true     // Enables wait for 10 seconds before starting
#define PRINT_SENSOR_DATA true // Requires SERIAL_LOGGING to be true

const int SENSOR_PRINT_INTERVAL = 250; // Interval to print sensor values using printSensorData (ms)

// Initialize objects
IMU imu;
ToF tof;
Motors motors;

const int NUMTURNS = 11; // Number of turns to make in the course
const float MAX_DIFF = 1.; // Threshold for difference between target and actual angle
const unsigned long MAX_SETTLE_TIME = 3000; // Max time given to PIDs to settle
const float TURN_ANGLE = 90.;
const float MAX_OVERSHOOT = 40.;
const float TURN_DIST_BIAS = 2.;
const float distToTurn[NUMTURNS] = {230., 230., 230., 500., 500., 500., 500., 780., 780., 780., 780.}; // Distances from wall (in mm) to turn at for every turn


// PID-related variables 
const int SAMPLE_TIME = 100; // Time between PID calculations (ms)
double turnKp = 10, turnKi = 0, turnKd = 1000;
double straightKp = 10, straightKi = 0, straightKd = 1000;
double turnInput, turnOutput; // Variables for turning PID control
double straightInput, straightOutput; // Variables for keeping straight PID control
double turnTarget = 0;
double straightTarget = 0;
PID turnPID(&turnInput, &turnOutput, &turnTarget, turnKp, turnKi, turnKd, DIRECT);
PID straightPID(&straightInput, &straightOutput, &straightTarget, straightKp, straightKi, straightKd, DIRECT);

//  Wheel speed variables
const float MIN_SPEED = 0.2;
const float CRUISE_SPEED = 0.8;
const float MIN_TURN_SPEED = 0.3;
const float MAX_TURN_SPEED = 0.8;

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
    turnPID.SetMode(AUTOMATIC); // Enable PIDs
    straightPID.SetMode(AUTOMATIC);

    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    printLineToSerial("Waiting for start button press"); // TODO: This should be in two stages for game day: press start button to go through calibration and/or pre-flight checks and then start again to go through the course
    while (digitalRead(START_BUTTON_PIN) == HIGH); // Wait until start button is pressed
    printLineToSerial("Start button pressed");

#if CALIBRATE_IMU
    printLineToSerial("Calibrating IMU...");
    for (int i = 0; i < 10; i++)
    {
        delay(1000);
        printLineToSerial("."); // Keep printing to serial to notify that stuff is still happening
    }
#endif
    imu.updateIMUState();
    imu.setZeroes(true, true, true); // Zero out yaw, pitch, and roll at start
}

void loop()
{
    // Continue straight to next turn
    runMotors(CRUISE_SPEED, CRUISE_SPEED);

    while (tof.getDist() > (distToTurn[turnsDone] + TURN_DIST_BIAS)) // While distance to wall is greater than the turning threshold
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
        adjustWheels(); // This function is empty
    }

    turnCorner();
    turnsDone += 1;
    imu.addToOffsets(-90, 0, 0); // Subtract 90 degrees from yaw offset so we're still close to zero degree heading after the turn, but accounting for the error
    // imu.setZeroes(true, true, true); // Alternatively, we could just zero out the IMU after turning but that will accumulate error in a different way

    // Stop and wait when done course
    if (turnsDone == NUMTURNS)
    {
        printLineToSerial("Done!");
        while (true);
    }
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
void turnCorner()
{
    // Stop car
    motors.brakeAllMotors();
    // Record initial values
    float initialAngle = imu.getIMUData().yaw;
    unsigned long initialTime = millis();
    // Set target angle for PID
    turnTarget = initialAngle + TURN_ANGLE;

    // Keep turning until within threshold of target angle and enough time to settle has elapsed
    // TODO: settling time should start only once the robot reaches the target angle for the first time
    while ((abs((initialAngle + TURN_ANGLE) - imu.getIMUData().yaw) > MAX_DIFF) &&  ((millis() - initialTime) > MAX_SETTLE_TIME))
    {
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
            printLineToSerial(String(millis()) + "imu yaw: " + String(imu.getIMUData().yaw));
        }
#endif
    }
    motors.brakeAllMotors();
}

void adjustWheels()
{
    // TODO: Write. Should bring in whatever code from previous version of function that is relevant
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
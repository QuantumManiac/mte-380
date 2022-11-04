
#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <Arduino.h>

////////////
// Pinout //
////////////
/* 
Pins on L298N boards that are used to control the motors
Motor location on car defined by F (front) or B (back) followed by L (left) or R (right)
ENABLE_PIN is used to enable the motor and set its power (if using PWM)
IN_A and IN_B are used to set the direction of the motor. 
A and B must be opposite logic levels (e.g. if IN_A is HIGH, IN_B is LOW). Swap these values to change direction.
*/

#define FL_ENABLE_PIN 8 
#define FL_IN_A_PIN 48
#define FL_IN_B_PIN 49
// Back-left motor
#define BL_ENABLE_PIN 9 
#define BL_IN_A_PIN 3
#define BL_IN_B_PIN 2
// Front-right motor
#define FR_ENABLE_PIN 10  
#define FR_IN_A_PIN 4
#define FR_IN_B_PIN 5
// Back-right motor
#define BR_ENABLE_PIN 11
#define BR_IN_A_PIN 7
#define BR_IN_B_PIN 6

////////////
// Config //
////////////

#define UNSAFE_PWM false    // Set true to use the max possible PWM value
#define MAX_PWM 255         // Maxiumum possible PWM value
#define MAX_SAFE_PWM 200    // Maxiumum "safe" PWM value. Going over this value will most likely overdrive the motors

#if UNSAFE_PWM 
    #define MOTOR_PWM_LIMIT MAX_PWM
#else
    #define MOTOR_PWM_LIMIT MAX_SAFE_PWM
#endif

///////////////
// Variables //
///////////////

enum MotorDirection {
    forward,
    backward
};

enum Motor {
    front_left,
    back_left,
    front_right,
    back_right,
    NUM_MOTORS // Always keep at end, tracks number of motors
};

int ENABLE_PINS[NUM_MOTORS] = {FL_ENABLE_PIN, BL_ENABLE_PIN, FR_ENABLE_PIN, BR_ENABLE_PIN};
int IN_A_PINS[NUM_MOTORS] = {FL_IN_A_PIN, BL_IN_A_PIN, FR_IN_A_PIN, BR_IN_A_PIN};
int IN_B_PINS[NUM_MOTORS] = {FL_IN_B_PIN, BL_IN_B_PIN, FR_IN_B_PIN, BR_IN_B_PIN};


class Motors {
    public:
        Motors();

        void initialize();

        void setMotorPower(Motor motor, float power);

        void setMotorDirection(Motor motor, MotorDirection direction);
};

#endif
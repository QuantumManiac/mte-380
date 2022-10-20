#line 1 "/home/chamath/Documents/Dev/GitHub/mte-380/software/libs/motors.h"

#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <Arduino.h>

// Pinout
#define FL_ENABLE_PIN 8 
#define FL_IN_A_PIN 0
#define FL_IN_B_PIN 1
// Back-left motor
#define BL_ENABLE_PIN 9 
#define BL_IN_A_PIN 2
#define BL_IN_B_PIN 3
// Front-right motor
#define FR_ENABLE_PIN 10  
#define FR_IN_A_PIN 4
#define FR_IN_B_PIN 5
// Back-right motor
#define BR_ENABLE_PIN 11
#define BR_IN_A_PIN 6
#define BR_IN_B_PIN 7

// Config
#define MAX_PWM 255
#define MAX_SAFE_PWM 200

#endif
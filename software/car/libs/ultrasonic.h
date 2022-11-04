#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include <Arduino.h>

// Pinout
#define TRIG_PIN 31
#define ECHO_PIN 30

// Consts
#define SPEED_OF_SOUND_CM_PER_US 0.0343


// Config

class Ultrasonic {
    public:
        Ultrasonic();

        void initialize();

        float getDist();
};

#endif
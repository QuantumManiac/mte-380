#ifndef _TOF_H_
#define _TOF_H_

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

// Config

class ToF {
    public:
        ToF();

        void initialize();

        int getDist();
    private:
        Adafruit_VL53L0X lox;
        int lastVal;

};

#endif
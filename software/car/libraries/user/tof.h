#ifndef _TOF_H_
#define _TOF_H_

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

// Config

class ToF {
    public:
        ToF();

        void initialize();

        float getDist();
    private:
        float lastVal;
};

#endif
#include "tof.h"

ToF::ToF() {
    Adafruit_VL53L0X lox;
    float lastVal = -1.;
}

/**
 * @brief initializes ultrasonic sensor
 * 
 */
void ToF::initialize() {
        Serial.println(lox.begin() ? "VL53L0X OK" : "VL53L0X ERROR");
        lox.startRangeContinuous();
}

/**
 * @brief Gets distance from ultrasonic sensor
 * 
 * @return float distance in cm
 */
float ToF::getDist() {
    float distance = lastVal;
    uint16_t measurement;
    
    if (lox.isRangeComplete()) {
        measurement = lox.readRange();
        if (measurement != 0xffff) {
            distance = lastVal = (measurement / 10.);
        }
    }
    
    return distance;
}
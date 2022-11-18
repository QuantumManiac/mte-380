#include "tof.h"

ToF::ToF() {
    Adafruit_VL53L0X lox = Adafruit_VL53L0X();
    float lastVal = -1.;
}

/**
 * @brief initializes ultrasonic sensor
 * 
 */
void ToF::initialize() {
    lox.startRangeContinuous();
    Serial.println("ToF Init");
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
            distance = lastVal = (measurement / 10);
        }
    }

    return distance;
}
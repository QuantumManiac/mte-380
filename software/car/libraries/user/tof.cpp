#include "tof.h"

ToF::ToF() {
    Adafruit_VL53L0X lox;
    float lastVal = 10000.;
}

/**
 * @brief initializes ultrasonic sensor
 * 
 */
void ToF::initialize() {
        Serial.println(lox.begin() ? "VL53L0X OK" : "VL53L0X ERROR");
}

/**
 * @brief Gets distance from ultrasonic sensor
 * 
 * @return float distance in cm
 */
float ToF::getDist() {
    float distance = -1;
    VL53L0X_RangingMeasurementData_t measurement;
    lox.rangingTest(&measurement, false);
    if (measurement.RangeStatus != 4) { // Phase failures have incorrect data
        distance = lastVal = (measurement.RangeMilliMeter / 10.);
    }
    
    return distance;
}
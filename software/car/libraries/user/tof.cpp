#include "tof.h"

ToF::ToF() {
    Adafruit_VL53L0X lox;
    int lastVal = 10000;
}

/**
 * @brief initializes ultrasonic sensor
 * 
 */
void ToF::initialize() {
        Serial.println(lox.begin() ? "VL53L0X OK" : "VL53L0X ERROR");
}

/**
 * @brief Gets distance from ToF sensor
 * 
 * @return float distance in mm
 */
int ToF::getDist() {
    int distance = lastVal;
    VL53L0X_RangingMeasurementData_t measurement;
    lox.rangingTest(&measurement, false);
    if (measurement.RangeStatus != 4) { // Phase failures have incorrect data
        distance = lastVal = measurement.RangeMilliMeter;
    }
    
    return distance;
}
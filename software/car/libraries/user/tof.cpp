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
    lox.begin(0x29, false, &Wire, Adafruit_VL53L0X::VL53L0X_Sense_config_t::VL53L0X_SENSE_LONG_RANGE);
        // Serial.println(lox.begin() ? "VL53L0X OK" : "VL53L0X ERROR");
}

/**
 * @brief Gets distance from ToF sensor
 * 
 * @return float distance in mm
 */
int ToF::getDist() {
    int distance = 10000;
    VL53L0X_RangingMeasurementData_t measurement;
    lox.rangingTest(&measurement, false);
    if (true) {
    // if (measurement.RangeStatus != 4) { // Phase failures have incorrect data
        distance = measurement.RangeMilliMeter;
    }
    
    return distance;
}
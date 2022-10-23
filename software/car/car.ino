#include "Wire.h"

// #include "libs/imu.cpp"
// #include "libs/motors.cpp"
#include "libs/ultrasonic.cpp"

Ultrasonic ultrasonic;

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    ultrasonic.initialize();
}

void loop()
{
    delay(500);
    Serial.println(ultrasonic.getDist());
}

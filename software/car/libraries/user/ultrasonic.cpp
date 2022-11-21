#include "ultrasonic.h"

Ultrasonic::Ultrasonic() {

}

/**
 * @brief initializes ultrasonic sensor
 * 
 */
void Ultrasonic::initialize() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    Serial.println("Ultrasonic Init");
}

/**
 * @brief Gets distance from ultrasonic sensor
 * 
 * @return float distance in cm
 */
float Ultrasonic::getDist() {
    long duration;
    float distance;

    // Clears the trigPin condition
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(ECHO_PIN, HIGH);
    // Calculating the distance
    distance = duration * SPEED_OF_SOUND_CM_PER_US / 2; // Speed of sound wave divided by 2 (go and back)
    
    return distance;
}
#include "motors.h"

Motors::Motors() {

}

/**
 * @brief Initializes motor controller
 * 
 */
void Motors::initialize() {

}

/**
 * @brief Sets direction of given motor
 * 
 * @param motor the motor to set direction for
 * @param direction the direction to set the motor to
 */
void Motors::setMotorDirection(Motor motor, MotorDirection direction) {
    if (direction == MotorDirection::forward) {
        digitalWrite(IN_A_PINS[motor], HIGH);
        digitalWrite(IN_B_PINS[motor], LOW);
    } else {
        digitalWrite(IN_A_PINS[motor], LOW);
        digitalWrite(IN_B_PINS[motor], HIGH);
    }
}

/**
 * @brief Sets power of given motor
 * 
 * @param motor the motor to set power of
 * @param power percentage power of max power to set motor to. Value between 0 and 1.
 */
void Motors::setMotorPower(Motor motor, float power) {
    analogWrite(ENABLE_PINS[motor], int(power * MOTOR_PWM_LIMIT));
}

/**
 * @brief Brakes given motor
 * 
 * @param motor the motor to brake
 */
void Motors::brakeMotor(Motor motor) {
    analogWrite(ENABLE_PINS[motor], 255);
    digitalWrite(IN_A_PINS[motor], LOW);
    digitalWrite(IN_B_PINS[motor], LOW);
}

/**
 * @brief Brakes all motors
 * 
 */
void Motors::brakeAllMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        brakeMotor((Motor)i);
    }
}
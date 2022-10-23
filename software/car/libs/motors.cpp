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
    bool isForward = direction == MotorDirection::forward;
    digitalWrite(IN_A_PINS[motor], isForward);
    digitalWrite(IN_B_PINS[motor], !isForward);
}

/**
 * @brief Sets power of the given motor
 * 
 * @param motor the motor to set power of
 * @param power percentage power of max power to set motor to. Value between 0 and 1.
 */
void Motors::setMotorPower(Motor motor, float power) {
    analogWrite(ENABLE_PINS[motor], int(power * MOTOR_PWM_LIMIT));
}
#include "imu.h"


IMU::IMU() {
  MPU9250_DMP imu;
  float yaw_offset = 0.;
  float pitch_offset = 0.;
  float roll_offset = 0.;
}

/**
 * @brief Initializes IMU
 * 
 */
void IMU::initialize() {
    while (!Serial); // wait for connection
    // verify connection
    Serial.println(imu.begin() == INV_SUCCESS ? "MPU9250 OK" : "MPU9250 ERROR");

    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
              DMP_FEATURE_GYRO_CAL, // Use gyro calibration
            10); // Set DMP FIFO rate to 10 Hz
}

/**
 * @brief The main AHRS loop to update the IMU data
 * 
 */
void IMU::updateIMUState() {
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
    }
  }
}

void IMU::setZeroes(bool yaw, bool pitch, bool roll) {
  if (yaw) {
    yaw_offset = -imu.yaw;
  }

  if (pitch) {
    pitch_offset = -imu.pitch;
  }

  if (roll) {
    roll_offset = -imu.roll;
  }
}

IMUData IMU::getIMUData() {
  float yaw = imu.yaw;
  float pitch = imu.pitch;
  float roll = imu.roll;

  if (yaw > 360) {
    yaw -= 360;
  }

  if (pitch > 360) {
    pitch -= 360;
  }

  if (roll > 360) {
    roll -= 360;
  }

  if (yaw < 0) {
    yaw += 360;
  }

  if (pitch < 0) {
    pitch += 360;
  }

  if (roll < 0) {
    roll += 360;
  }

  IMUData data = {yaw, pitch, roll};
  return data;
}


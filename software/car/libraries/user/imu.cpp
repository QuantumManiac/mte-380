#include "imu.h"

const float FULL_ROTATION = 360.0;

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
    imu.begin();
    // Serial.println(imu.begin() == INV_SUCCESS ? "MPU9250 OK" : "MPU9250 ERROR");

    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
              DMP_FEATURE_GYRO_CAL, // Use gyro calibration
            50); // Set DMP FIFO rate to 50 Hz
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
  updateIMUState();
  if (yaw) {
    yaw_offset = -(imu.yaw);
  }

  if (pitch) {
    pitch_offset = -(imu.pitch);
  }

  if (roll) {
    roll_offset = -(imu.roll);
  }
}

void IMU::addToOffsets(float yaw, float pitch, float roll) {
  updateIMUState();
  yaw_offset += yaw;
  pitch_offset += pitch;
  roll_offset += roll;
}

IMUData IMU::getIMUData() {
  updateIMUState();
  // Bound heading values to [-360, 360]
  float yaw = fmod(imu.yaw + yaw_offset, FULL_ROTATION);
  float pitch = fmod(imu.pitch + pitch_offset, FULL_ROTATION);
  float roll = fmod(imu.roll + roll_offset, FULL_ROTATION);

  // Bound heading values to [-180, 180]
  // Map [(180, 360) -> (-180, 0)]
  if (yaw > 180) {
    yaw = -(FULL_ROTATION - yaw);
  }

  if (pitch > 180) {
    pitch = -(FULL_ROTATION - pitch);
  }

  if (roll > 180) {
    roll = -(FULL_ROTATION - roll);
  }

  // Map [(-360, -180) -> (0, 180)]
  if (yaw <= -180) {
    yaw = (FULL_ROTATION + yaw);
  }

  if (pitch <= -180) {
    pitch = (FULL_ROTATION + pitch);
  }

  if (roll <= -180) {
    roll = (FULL_ROTATION + roll);
  }

  IMUData data = {yaw, pitch, roll};
  return data;
}


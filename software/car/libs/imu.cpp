#include "imu.h"


IMU::IMU() {
  MPU9250_DMP imu;
  yaw_offset = 0;
  pitch_offset = 0;
  roll_offset = 0;
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
  float raw_yaw = imu.yaw;
  float raw_pitch = imu.pitch;
  float raw_roll = imu.roll;

  float pitch = -(raw_pitch - 360); 
  
  // Process roll
  float roll = raw_roll;
  if (roll > 180) roll -= 360;

  // Process pitch
  pitch = raw_pitch - 360;

  IMUData data = {imu.yaw, imu.pitch, imu.roll};
  return data;
}


#include "imu.h"


IMU::IMU() {
    // Variables for AHRS loop timing
    tau = 0.98;
    data = {0,0,0};
}

/**
 * @brief Initializes IMU
 * 
 */
void IMU::initialize() {
    while (!Serial); // wait for connection
    // Initialize MPU9250 device
    setup_mpu_6050_registers();
    // verify connection
    Serial.println("MPU9250 OK");
    loopTimer = micros();
    loopTimer2 = micros();
}

/**
 * @brief The main AHRS loop to update the IMU data
 * 
 */
void IMU::updateIMUState() {
    freq = 1/((micros() - loopTimer2) * 1e-6);
    loopTimer2 = micros();
    dt = 1/freq;

    // Read the raw acc data from MPU-6050
    read_mpu_6050_data();

    // Subtract the offset calibration value
    gyro_x -= G_off[0];
    gyro_y -= G_off[1];
    gyro_z -= G_off[2];

    // Convert to instantaneous degrees per second
    rotation_x = (double)gyro_x / (double)scaleFactorGyro;
    rotation_y = (double)gyro_y / (double)scaleFactorGyro;
    rotation_z = (double)gyro_z / (double)scaleFactorGyro;

    // Convert to g force
    accel_x = (double)acc_x / (double)scaleFactorAccel;
    accel_y = (double)acc_y / (double)scaleFactorAccel;
    accel_z = (double)acc_z / (double)scaleFactorAccel;

    // Complementary filter
    accelPitch = atan2(accel_y, accel_z) * RAD_TO_DEG;
    accelRoll = atan2(accel_x, accel_z) * RAD_TO_DEG;
    accelYaw = atan2(accel_x, accel_y) * RAD_TO_DEG;

    data.pitch = (tau)*(data.pitch + rotation_x*dt) + (1-tau)*(accelPitch);
    data.roll = (tau)*(data.roll - rotation_y*dt) + (1-tau)*(accelRoll);
    // data.yaw = (tau)*(data.yaw + rotation_z*dt) + (1-tau)*(accelYaw);
    data.yaw = gyroYaw;

    if (data.yaw < 0) data.yaw += 360.0;
    if (data.yaw >= 360.0) data.yaw -= 360.0;

    gyroPitch += rotation_x*dt;
    gyroRoll -= rotation_y*dt;
    gyroYaw += rotation_z*dt;
}

/**
 * @brief Gets IMU data
 * 
 * @return IMUData 
 */
IMUData IMU::getIMUData() {
    return data;
}

void IMU::setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure the accelerometer
  // Wire.write(0x__);
  // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();

  // Configure the gyro
  // Wire.write(0x__);
  // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void IMU::read_mpu_6050_data() {
  // Subroutine for reading the raw data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  // Read data --> Temperature falls between acc and gyro registers
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() <<8 | Wire.read();
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();
}



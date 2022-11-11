#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include <Wire.h>
#include "imu/I2Cdev.cpp"
#include "imu/MPU9250.cpp"

////////////
// Config //
////////////
float G_off[3] = {121.6, -71.0, 53.0}; //raw offsets, determined for gyro at rest

struct IMUData{
    float yaw;
    float pitch;
    float roll;
};

class IMU {
    private:
        IMUData data = {0,0,0};  
        long loopTimer, loopTimer2;
        int temperature;
        double accelRoll, accelPitch, accelYaw;
        long acc_x, acc_y, acc_z;
        double accel_x, accel_y, accel_z;
        double gyroRoll, gyroPitch, gyroYaw;
        int gyro_x, gyro_y, gyro_z;
        long gyro_x_cal, gyro_y_cal, gyro_z_cal;
        double rotation_x, rotation_y, rotation_z;
        double freq, dt;
        double tau;

        // 250 deg/s --> 131.0, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
        long scaleFactorGyro = 65.5;

        // 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
        long scaleFactorAccel = 8192;

        void setup_mpu_6050_registers();
        void read_mpu_6050_data();
    
    public:
        IMU();

        void initialize();

        void updateIMUState();

        IMUData getIMUData();
};


#endif
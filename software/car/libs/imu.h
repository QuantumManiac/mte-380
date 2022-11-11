#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>

////////////
// Config //
////////////
struct IMUData{
    float yaw;
    float pitch;
    float roll;
};

class IMU {
    private:
        MPU9250_DMP imu;
        float yaw_offset;
        float pitch_offset;
        float roll_offset;
    public:
        IMU();

        void initialize();

        void updateIMUState();

        void setZeroes(yaw, pitch, roll);

        IMUData getIMUData();
};


#endif
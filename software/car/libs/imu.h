#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include <Wire.h>
#include "imu/I2Cdev.cpp"
#include "imu/MPU9250.cpp"

////////////
// Config //
////////////
// CALIBRATION PARAMETERS
// accel offsets and correction matrix
float A_B[3] 
  {  117.13,  299.31, -580.83};
float A_Ainv[3][3]
  {{  0.60697,  0.00080,  0.00120},
  {  0.00080,  0.61477,  0.00654},
  {  0.00120,  0.00654,  0.59872}};
  
// mag offsets and correction matrix
float M_B[3]
  {   68.94,   37.60,  -53.94};
float M_Ainv[3][3]
  {{  1.15765, -0.00827,  0.01936},
  { -0.00827,  1.20352,  0.05185},
  {  0.01936,  0.05185,  1.07880}};
  
float G_off[3] = {124.6, -68.0, 40.0}; //raw offsets, determined for gyro at rest

#define gscale (250./32768.0)*(PI/180.0)  //gyro default 250 LSB per d/s -> rad/s

//char s[60]; //snprintf buffer

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// with MPU-9250, angles start oscillating strongly at Kp=50. Ki does not seem to help and is not required.
#define Kp 25.0
#define Ki 0.0


struct IMUData{
    float yaw;
    float pitch;
    float roll;
};

class IMU {
    private:
        unsigned long now, last;
        float deltat;
        unsigned long now_ms, last_ms;
        unsigned long print_ms;
        
        MPU9250 accelgyro;
        I2Cdev   I2C_M;
        
        IMUData data = {0,0,0};
        
        //raw data scaled as vector
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t mx, my, mz;
        float Axyz[3];
        float Gxyz[3];
        float Mxyz[3];

        float q[4];

        void getMPUScaled();

        void mahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

        float vectorDot(float a[3], float b[3]);

        void vectorNormalize(float a[3]);   
    
    public:
        IMU();

        void initialize();

        void updateIMUState();

        IMUData getIMUData();
};


#endif
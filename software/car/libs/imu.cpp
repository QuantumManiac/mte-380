#include "imu.h"


IMU::IMU() {
    // Variables for AHRS loop timing
    unsigned long now = 0, last = 0; //micros() timers
    float deltat = 0;  //loop time in seconds
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
    IMUData data = {0,0,0};

}

/**
 * @brief Initializes IMU
 * 
 */
void IMU::initialize() {
    while (!Serial); // wait for connection

    // Initialize MPU9250 device
    accelgyro.initialize();
    // verify connection
    Serial.println(accelgyro.testConnection() ? "MPU9250 OK" : "MPU9250 ??");
    last = micros();
}

/**
 * @brief The main AHRS loop to update the IMU data
 * 
 */
void IMU::updateIMUState() {
    getMPUScaled();
    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    // correct for differing accelerometer and magnetometer alignment by circularly permuting mag axes
    mahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                            Mxyz[1], Mxyz[0], -Mxyz[2], deltat);

    // Tait-Bryan angles.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North
    // (or true North if corrected for local declination, looking down on the sensor
    // positive yaw is counterclockwise, which is not conventional for navigation.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
    
    // Warning: strictly valid only for approximately level movement. This code WILL MALFUNCTION
    // for certain orientations! DEMONSTRATION VERSION only.
    
    float roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    float pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    float yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw   *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    yaw = -yaw + 15.0;
    if (yaw < 0) yaw += 360.0;
    if (yaw >= 360.0) yaw -= 360.0;
    data = {yaw, pitch, roll};
}

/**
 * @brief Gets IMU data
 * 
 * @return IMUData 
 */
IMUData IMU::getIMUData() {
    return data;
}

void IMU::getMPUScaled() {
    float temp[3];
    int i;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = ((float) gx - G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float) gy - G_off[1]) * gscale;
    Gxyz[2] = ((float) gz - G_off[2]) * gscale;

    Axyz[0] = (float) ax;
    Axyz[1] = (float) ay;
    Axyz[2] = (float) az;
    //apply offsets (bias) and scale factors from Magneto
    for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
    Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
    Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
    Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
    vectorNormalize(Axyz);

    Mxyz[0] = (float) mx;
    Mxyz[1] = (float) my;
    Mxyz[2] = (float) mz;
    //apply offsets and scale factors from Magneto
    for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
    Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
    Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
    Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
    vectorNormalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// reference vectors are Up (Acc) and West (Acc cross Mag)
// sjr 12/2020
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
void IMU::mahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat) {
    // Vector to hold integral error for Mahony method
    static float eInt[3] = {0.0, 0.0, 0.0};
    static unsigned long start = 0;
    static char first = 1;

    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, hz;  //observed West vector W = AxM
    float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Measured horizon vector = a x m (in body frame)
    hx = ay * mz - az * my;
    hy = az * mx - ax * mz;
    hz = ax * my - ay * mx;
    // Normalise horizon vector
    norm = sqrt(hx * hx + hy * hy + hz * hz);
    if (norm == 0.0f) return; // Handle div by zero

    norm = 1.0f / norm;
    hx *= norm;
    hy *= norm;
    hz *= norm;

    // Estimated direction of Up reference vector
    ux = 2.0f * (q2q4 - q1q3);
    uy = 2.0f * (q1q2 + q3q4);
    uz = q1q1 - q2q2 - q3q3 + q4q4;

    // estimated direction of horizon (West) reference vector
    wx = 2.0f * (q2q3 + q1q4);
    wy = q1q1 - q2q2 + q3q3 - q4q4;
    wz = 2.0f * (q3q4 - q1q2);

    // Error is cross product between estimated direction and measured direction of the reference vectors

    ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
    ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
    ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

    if (Ki > 0.0f) {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
        // Apply I feedback
        gx += Ki * eInt[0];
        gy += Ki * eInt[1];
        gz += Ki * eInt[2];
    }

    // Apply P feedback
    gx = gx + Kp * ex;
    gy = gy + Kp * ey;
    gz = gz + Kp * ez;

    //update quaternion with integrated contribution
    // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
    gx = gx * (0.5*deltat); // pre-multiply common factors
    gy = gy * (0.5*deltat);
    gz = gz * (0.5*deltat);
    float qa = q1;
    float qb = q2;
    float qc = q3;
    q1 += (-qb * gx - qc * gy - q4 * gz);
    q2 += (qa * gx + qc * gz - q4 * gy);
    q3 += (qa * gy - qb * gz + q4 * gx);
    q4 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

float IMU::vectorDot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void IMU::vectorNormalize(float a[3])
{
  float mag = sqrt(vectorDot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}




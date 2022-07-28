// Header file for IMU quaternion class

#ifndef Imu_head
#define Imu_head

#include <cmath>

class Imu
{
public:
    double quat[4]; // quaternion array initialization
    double dT;      // time step initialization

    Imu(double *quat_init, double dT); // class constructor

    void update(double *data); // updating function
private:
    void quatMultiply(double *quat, double *b); // quaternion multiplication
    void quatNormalize(double *quat);           // normaliztion quaternion
};

#endif
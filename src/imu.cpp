// Implementation file for IMU header
#include <iostream>
#include "imu.h"

using namespace std;

Imu::Imu(double *quat_init, double dT) : dT(dT) // constructor declaration
{

    quat[0] = quat_init[0];
    quat[1] = quat_init[1];
    quat[2] = quat_init[2];
    quat[3] = quat_init[3];
}

void Imu::update(double *data)
{
    double b[4];
    b[0] = 1;
    b[1] = 0.5 * data[0] * dT;
    b[2] = 0.5 * data[1] * dT;
    b[3] = 0.5 * data[2] * dT;
    quatMultiply(quat, b);
    quatNormalize(quat);
}

void Imu::quatMultiply(double *quat, double *b)
{
    double a_ssm[4][4];

    a_ssm[0][0] = quat[0];
    a_ssm[0][1] = -quat[1];
    a_ssm[0][2] = -quat[2];
    a_ssm[0][3] = -quat[3];
    a_ssm[1][0] = quat[1];
    a_ssm[1][1] = quat[0];
    a_ssm[1][2] = -quat[3];
    a_ssm[1][3] = quat[2];
    a_ssm[2][0] = quat[2];
    a_ssm[2][1] = quat[3];
    a_ssm[2][2] = quat[0];
    a_ssm[2][3] = -quat[1];
    a_ssm[3][0] = quat[3];
    a_ssm[3][1] = -quat[2];
    a_ssm[3][2] = quat[1];
    a_ssm[3][3] = quat[0];

    quat[0] = a_ssm[0][0] * b[0] + a_ssm[0][1] * b[1] + a_ssm[0][2] * b[2] + a_ssm[0][3] * b[3];
    quat[1] = a_ssm[1][0] * b[0] + a_ssm[1][1] * b[1] + a_ssm[1][2] * b[2] + a_ssm[1][3] * b[3];
    quat[2] = a_ssm[2][0] * b[0] + a_ssm[2][1] * b[1] + a_ssm[2][2] * b[2] + a_ssm[2][3] * b[3];
    quat[3] = a_ssm[3][0] * b[0] + a_ssm[3][1] * b[1] + a_ssm[3][2] * b[2] + a_ssm[3][3] * b[3];
}

void Imu::quatNormalize(double *quat)
{
    double quatmag = sqrt(pow(quat[0], 2) + pow(quat[1], 2) + pow(quat[2], 2) + pow(quat[3], 2));

    quat[0] = quat[0] / quatmag;
    quat[1] = quat[1] / quatmag;
    quat[2] = quat[2] / quatmag;
    quat[3] = quat[3] / quatmag;
}
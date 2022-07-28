// Homework 6 Main Function
#include <iostream>
#include <fstream>
#include <string>
#include "imu.h"

using namespace std;

int main()
{
    string i_file = "/home/noahfireball1/softSys/hw-nsm0014/data/imu_data.txt";       // Ingesting data from imu
    string inputs_file = "/home/noahfireball1/softSys/hw-nsm0014/data/gyro_data.txt"; // for debugging, to see if the update function is receiving the correct measurements
    string o_file = "/home/noahfireball1/softSys/hw-nsm0014/data/out_data.txt";       // quaternion output file

    double acc[3], gyro[3], mag[3];        // Defining measurement variables from IMU
    double quat[4] = {1.0, 0.0, 0.0, 0.0}; // Initial quaternion array
    double dT = 1.0 / 128.0;               // Time step

    Imu imu(quat, dT); // Constructor using above given parameters

    ifstream infile(i_file);        // imu data.txt
    ofstream outfile(o_file);       // out data.txt
    ofstream outfile1(inputs_file); // gyro data.txt

    while (infile >> acc[0] >> acc[1] >> acc[2] >> gyro[0] >> gyro[1] >> gyro[2] >> mag[0] >> mag[1] >> mag[2]) // reading the order of the imu text file
    {
        outfile << imu.quat[0] << "\t"; // writing the calculated quaternions to the output file
        outfile << imu.quat[1] << "\t";
        outfile << imu.quat[2] << "\t";
        outfile << imu.quat[3] << endl;

        outfile1 << gyro[0] << "\t"; // writing the ingested gyro measurements
        outfile1 << gyro[1] << "\t";
        outfile1 << gyro[2] << endl;

        imu.update(gyro); // updating function
    }
    infile.close();
    outfile.close();
    return 0;
}
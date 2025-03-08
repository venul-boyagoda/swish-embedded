#ifndef mahony_h
#define mahony_h

extern volatile float qq0, qq1, qq2, qq3; // quaternion of sensor frame relative to auxiliary frame

// Function prototype
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif
